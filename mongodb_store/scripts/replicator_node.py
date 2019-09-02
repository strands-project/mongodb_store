#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import
"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import actionlib
import pymongo
try:
    from Queue import Queue
except ImportError:
    from queue import Queue
import os
import re
import shutil
import subprocess
from bson import json_util
import sys
import time
from threading import Thread, Lock
from mongodb_store_msgs.msg import MoveEntriesAction, MoveEntriesFeedback
from datetime import datetime

import mongodb_store.util
from mongodb_store_msgs.msg import MoveEntriesAction, MoveEntriesFeedback
MongoClient = mongodb_store.util.import_MongoClient()


class Process(object):
    def __init__(self, cmd):
        self.lock = Lock()
        self.cmd = cmd
        self.process = None
        self.threads = []

    def _message_callback(self, stream, callback):
        buf = str()
        for line in iter(stream.readline, b''):
            callback(line.strip())

    def __del__(self):
        self.shutdown()

    def start(self):
        with self.lock:
            self.process = subprocess.Popen(
                self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                bufsize=1, close_fds='posix' in sys.builtin_module_names,
                env=os.environ.copy())

            self.threads = [
                Thread(target=self._message_callback,
                       args=(self.process.stdout, self.on_stdout)),
                Thread(target=self._message_callback,
                       args=(self.process.stderr, self.on_stderr))
            ]
            for th in self.threads:
                th.daemon = True
                th.start()

            rospy.loginfo('[{}] started with pid={}'.format(self.cmd[0], self.process.pid))

            self.on_start()

    def shutdown(self):
        with self.lock:
            cmdstr = ' '.join(self.cmd)
            if self.process is not None:
                if self.process.poll() is None:
                    rospy.loginfo('Terminating the process "{}"'.format(cmdstr))
                    self.process.terminate()
                    self.wait(timeout=15.0)
                if self.process.poll() is None:
                    rospy.loginfo('Escalated to SIGKILL')
                    self.process.kill()
                    self.wait(timeout=5.0)
                if self.process.poll() is None:
                    rospy.logerr('[{}] The process could not be killed. (pid={}).'.format(
                        self.cmd[0], self.process.pid))
                else:
                    if self.process.poll() == 0:
                        rospy.loginfo('[{}] The process (pid={}) was successfully shutdown (code={})'.format(
                            self.cmd[0], self.process.pid, self.process.returncode))
                    else:
                        rospy.logerr('[{}] The process (pid={}) was shutdown abnormaly (code={})'.format(
                            self.cmd[0], self.process.pid, self.process.returncode))

                    for th in self.threads:
                        try:
                            th.join()
                        except:
                            pass
                    self.threads = []
                    self.process = None
                    self.on_shutdown()

    def wait(self, timeout=None):
        if self.process is None:
            return
        if self.process.poll() is None:
            end_time = None
            if timeout is not None:
                end_time = time.time() + timeout
            while self.process.poll() is None:
                now = time.time()
                if end_time is not None and now > end_time:
                    return
                rospy.sleep(0.1)
            self.shutdown()

    def poll(self):
        if self.process is not None:
            return self.process.poll()
        else:
            return -1

    def on_start(self):
        pass

    def on_shutdown(self):
        pass

    def on_stdout(self, msg):
        rospy.loginfo('[{}] {}'.format(self.cmd[0], msg))

    def on_stderr(self, msg):
        rospy.logerr('[{}] {}'.format(self.cmd[0], msg))


class MongoProcess(Process):
    _regex = re.compile(r'^([0-9]+/[0-9]+)$')

    def on_start(self):
        self._progress = 0.0
        super(MongoProcess, self).on_start()

    def on_output(self, msg):
        super(MongoProcess, self).on_output(msg)
        try:
            progress = filter(self._regex.match, msg.split())
            current, total = progress[0].split('/')
            self._progress = float(current) / float(total) * 100.0
        except:
            pass

    @property
    def progress(self):
        self._progress


class MongoDumpProcess(MongoProcess):
    def __init__(self, host, port, db, collection, dump_path, less_time=None, query=None):
        cmd = [
            'mongodump', '--verbose', '-o', dump_path,
            '--host', host, '--port', str(port),
            '--db', db, '--collection', collection,
        ]

        if query is None:
            query = {}
        if less_time is not None:
            query.update({
                '_meta.inserted_at': {'$lt': datetime.utcfromtimestamp(less_time.to_sec())}
            })

        if query:
            query = json_util.dumps(query)
            cmd += ['--query', query]

        super(MongoDumpProcess, self).__init__(cmd=cmd)


class MongoRestoreProcess(MongoProcess):
    def __init__(self, host, port, dump_path, db=None, collection=None):
        cmd = [
            'mongorestore', '--verbose', '--host', host, '--port', str(port),
        ]
        if db is not None:
            cmd += ['--db', db]
        if collection is not None:
            cmd += [ '--collection', collection]
        cmd += [dump_path]
        super(MongoRestoreProcess, self).__init__(cmd=cmd)


class Replicator(object):
    def __init__(self):

        # don't start up until master is there
        use_daemon = rospy.get_param('mongodb_use_daemon', False)


        if use_daemon:
            self.master_db_host = rospy.get_param('mongodb_host')
            self.master_db_port = rospy.get_param('mongodb_port')
            is_daemon_alive = mongodb_store.util.check_connection_to_mongod(self.master_db_host, self.master_db_port)
            if not is_daemon_alive:
                raise Exception("No Daemon?")
        else:
            if not mongodb_store.util.wait_for_mongo():
                raise Exception("No Datacentre?")
            self.master_db_host = rospy.get_param('mongodb_host')
            self.master_db_port = rospy.get_param('mongodb_port')
        # this is just a test, connections are remade every call for long-running processes
        master, extras = self.make_connections()
        if master is None:
            raise Exception("No master datacentre found using mongodb_host and mongodb_port")

        self.server = actionlib.SimpleActionServer('move_mongodb_entries', MoveEntriesAction, self.move_entries, False)
        self.server.register_preempt_callback(self.do_cancel)
        self.restore_process = None
        self.dump_process = None
        self.server.start()
        self.dump_path = rospy.get_param("~replicator_dump_path", '/tmp/mongodb_replicator')
        rospy.loginfo("Replicator node dumping to %s" % self.dump_path)

        self.make_path()
        self.remove_path()


    def make_path(self):
        if not os.path.isdir(self.dump_path):
            os.makedirs(self.dump_path)
        elif not os.access(self.dump_path, os.W_OK):
            raise Exception('Cannot write to dump path: %s' % self.dump_path)

    def remove_path(self):
        shutil.rmtree(self.dump_path)

    def make_connections(self):
        master = None
        try:
            master = MongoClient(self.master_db_host, self.master_db_port)
        except pymongo.errors.ConnectionFailure:
            rospy.logwarn('Could not connect to master datacentre at %s:%s' % (
                self.master_db_host, self.master_db_port))
            return None, None


        extras = rospy.get_param('mongodb_store_extras', [])
        extra_clients = []
        for extra in extras:
            try:
                extra_clients.append(MongoClient(extra[0], extra[1]))
            except pymongo.errors.ConnectionFailure as e:
                rospy.logwarn('Could not connect to extra datacentre at %s:%s' % (extra[0], extra[1]))


        rospy.loginfo('Replicating content from %s:%s to a futher %s datacentres', self.master_db_host, self.master_db_port, len(extra_clients))

        return master, extra_clients

    def move_entries(self, goal):

        # create place to put temp stuf
        self.make_path()
        # don't use the connections, just sanity check their existence
        master, extras = self.make_connections()
        if len(extras) == 0:
            rospy.logwarn('No datacentres to move to, not performing move')
            self.server.set_aborted()
            return

        completed = []
        feedback = MoveEntriesFeedback(completed=completed)

        less_time_time = rospy.get_rostime() - goal.move_before

        query = mongodb_store.util.string_pair_list_to_dictionary(goal.query)

        for collection in goal.collections.data:
            self.do_dump(collection, master, less_time_time,
                         db=goal.database, query=query)

        self.do_restore(extras, db=goal.database)

        if goal.delete_after_move:
            for collection in goal.collections.data:
                self.do_delete(collection, master, less_time_time,
                               db=goal.database, query=query)

        # clean up
        self.remove_path()

        if self.server.is_preempt_requested():
            self.server.set_preempted()
        else:
            self.server.set_succeeded()

    def do_restore(self, extras, db='message_store'):
        """restore collection to extras"""
        for extra in extras:
            if self.server.is_preempt_requested():
                break
            try:
                host, port = extra.address  # pymongo >= 3.0
            except TypeError:
                host, port = extra.host, extra.port
            except pymongo.errors.ServerSelectionTimeoutError:
                rospy.logerr('Failed to connect to the extra server {}'.format(extra))
                continue
            self.restore_process = MongoRestoreProcess(host=host, port=port, dump_path=self.dump_path)
            self.restore_process.start()
            self.restore_process.wait()
            self.restore_process = None

    def do_delete(self, collection, master, less_time_time=None, db='message_store', query=None):
        coll = master[db][collection]
        spec = dict()
        if query is not None:
            spec.update(query)
        if less_time_time is not None:
            spec.update({
                "_meta.inserted_at": {"$lt": datetime.utcfromtimestamp(less_time_time.to_sec())}
            })
        coll.remove(spec)

    def do_dump(self, collection, master, less_time_time=None, db='message_store', query=None):
        """dump collection"""
        try:
            host, port = master.address  # pymongo >= 3.0
        except TypeError:
            host, port = master.host, master.port
        except pymongo.errors.ServerSelectionTimeoutError:
            rospy.logerr('Failed to connect to the master server {}'.format(master))
            return False

        self.dump_process = MongoDumpProcess(host=host, port=port, db=db, collection=collection,
                                             dump_path=self.dump_path,
                                             less_time=less_time_time, query=query)
        self.dump_process.start()
        self.dump_process.wait()
        self.dump_process = None

    def do_cancel(self):
        if self.dump_process:
            self.dump_process.shutdown()
            self.dump_process = None
        if self.restore_process:
            self.restore_process.shutdown()
            self.restore_process = None

if __name__ == '__main__':
    rospy.init_node("mongodb_replicator")
    store = Replicator()
    rospy.spin()
