import rclpy
import subprocess
import sys
import os
import re
import errno
from rclpy.duration import Duration
import threading

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
import shutil
import pymongo

import mongodb_store.util

if not mongodb_store.util.check_for_pymongo():
    sys.exit(1)

MongoClient = mongodb_store.util.import_MongoClient()


def is_socket_free(host, port):
    import socket

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = sock.connect_ex((host, port))
    return result != 0


class MongoServer(rclpy.node.Node):
    def __init__(self):
        # TODO: This should be anonymous, but ROS2 doesn't allow for that natively
        super().__init__("mongodb_server")

        # Has the db already gone down, before the ros node?
        self._gone_down = False

        self._ready = False  # is the db ready: when mongo says "waiting for connection"

        self.test_mode = self.declare_parameter(
            "test_mode", False, descriptor=ParameterDescriptor(description="")
        ).value
        self.repl_set = self.declare_parameter(
            "repl_set", "", descriptor=ParameterDescriptor(description="")
        ).value
        self.bind_to_host = self.declare_parameter(
            "bind_to_host", False, descriptor=ParameterDescriptor(description="")
        ).value

        if self.test_mode:
            import random

            default_host = "localhost"
            default_port = random.randrange(49152, 65535)

            count = 0
            while not is_socket_free(default_host, default_port):
                default_port = random.randrange(49152, 65535)
                count += 1
                if count > 100:
                    self.get_logger().error(
                        "Can't find a free port to run the test server on."
                    )
                    sys.exit(1)

            self.default_path = "/tmp/ros_mongodb_store_%d" % default_port
            os.mkdir(self.default_path)
        else:
            default_host = "localhost"
            default_port = 27017
            self.default_path = "/opt/ros/mongodb_store"

        # Get the database path
        self._db_path = self.declare_parameter(
            "database_path",
            self.default_path,
            descriptor=ParameterDescriptor(description=""),
        ).value
        is_master = self.declare_parameter(
            "master", True, descriptor=ParameterDescriptor(description="")
        ).value

        if is_master:
            # TODO: These used to be global params
            self._mongo_host = self.declare_parameter(
                "mongodb_host",
                default_host,
                descriptor=ParameterDescriptor(description=""),
            ).value
            self._mongo_port = self.declare_parameter(
                "mongodb_port",
                default_port,
                descriptor=ParameterDescriptor(description=""),
            ).value
        else:
            self._mongo_host = self.declare_parameter(
                "host", descriptor=ParameterDescriptor(description="")
            ).value
            self._mongo_port = self.declare_parameter(
                "port", descriptor=ParameterDescriptor(description="")
            ).value

        self.get_logger().info(
            "Mongo server address: " + self._mongo_host + ":" + str(self._mongo_port)
        )

        # Check that mongodb is installed
        try:
            mongov = subprocess.check_output(["mongod", "--version"])
            match = re.search("db version v(\d+\.\d+\.\d+)", mongov.decode("utf-8"))
            self._mongo_version = match.group(1)
        except subprocess.CalledProcessError:
            self.get_logger().error(
                'Can\'t find MongoDB executable. Is it installed?\nInstall it with  "sudo apt install mongodb"'
            )
            sys.exit(1)
        self.get_logger().info("Found MongoDB version " + self._mongo_version)

        # Check that the provided db path exists.
        if not os.path.exists(self._db_path):
            self.get_logger().error(
                f"Can't find database at supplied path {self._db_path}. If this is a new DB, create it as an empty "
                f"directory."
            )
            sys.exit(1)

        # Advertise ros services for db interaction
        self._shutdown_srv = self.create_service(
            Empty, "/datacentre/shutdown", self._shutdown_srv_cb
        )
        self._wait_ready_srv = self.create_service(
            Empty, "/datacentre/wait_ready", self._wait_ready_srv_cb
        )

        self.mongo_thread = threading.Thread(target=self._mongo_loop)
        self.mongo_thread.start()

    def _mongo_loop(self):

        # Blocker to prevent Ctrl-C being passed to the mongo server
        def block_mongo_kill():
            os.setpgrp()

        #            signal.signal(signal.SIGINT, signal.SIG_IGN)

        # cmd = ["mongod","--dbpath",self._db_path,"--port",str(self._mongo_port),"--smallfiles","--bind_ip","127.0.0.1"]
        cmd = ["mongod", "--dbpath", self._db_path, "--port", str(self._mongo_port)]

        if self.bind_to_host:
            cmd.append("--bind_ip")
            cmd.append(self._mongo_host)
        else:
            cmd.append("--bind_ip")
            cmd.append("0.0.0.0")

        if self.repl_set:
            cmd.append("--replSet")
            cmd.append(self.repl_set)

        self.get_logger().info(f"Running command {' '.join(cmd)}")
        self._mongo_process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, preexec_fn=block_mongo_kill
        )

        while self._mongo_process.poll() is None:  # and rclpy.ok():
            try:
                stdout = self._mongo_process.stdout.readline().decode("utf-8")
            except IOError as e:  # probably interupt because shutdown cut it up
                if e.errno == errno.EINTR:
                    continue
                else:
                    raise
            if stdout is not None:
                # if stdout.find("ERROR") != -1:
                #     self.get_logger().error(stdout.strip())
                # else:
                #     self.get_logger().info(stdout.strip())

                if not self._ready and stdout.find("mongod startup complete") != -1:
                    self._ready = True
                    if self.repl_set:
                        try:
                            self.initialize_repl_set()
                        except Exception as e:
                            self.get_logger().warning(
                                f"initialzing replSet failed: {e}"
                            )

        if not rclpy.ok():
            self.get_logger().error("MongoDB process stopped!")

        if self._mongo_process.returncode != 0:
            self.get_logger().error(
                "Mongo process error! Exit code=" + str(self._mongo_process.returncode)
            )

        self._gone_down = True

    def _shutdown_srv_cb(
        self, request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        # Calling shutdown exits the spin on the node.
        rclpy.shutdown()
        return Empty.Response()

    def _wait_ready_srv_cb(
        self, request: Empty.Request, resp: Empty.Response
    ) -> Empty.Response:
        while not self._ready:
            self.get_clock().sleep_for(Duration(seconds=0.1))
        return Empty.Response()

    def initialize_repl_set(self):
        c = pymongo.Connection(
            f"{self._mongo_host}:{self._mongo_port}", slave_okay=True
        )
        c.admin.command("replSetInitiate")
        c.close()


def main():
    rclpy.init()
    try:
        server = MongoServer()
        rclpy.spin(server, executor=MultiThreadedExecutor())
        server.destroy_node()
        if rclpy.ok():
            # If the shutdown srv is called calling this again will cause a crash
            rclpy.shutdown()
    finally:
        # TODO: The context on_shutdown doesn't seem to work, so moving that code here
        server.get_logger().info("Shutting down datacentre")
        server._ready = False
        if server._gone_down:
            server.get_logger().warning(
                "It looks like Mongo already died. Watch out as the DB might need recovery time at next run."
            )
            return
        try:
            c = MongoClient(host=server._mongo_host, port=server._mongo_port)
        except pymongo.errors.ConnectionFailure:
            c = None
        try:
            if c is not None:
                c.admin.command("shutdown")
        except pymongo.errors.AutoReconnect:
            pass

        if server.test_mode:  # remove auto-created DB in the /tmp folder
            try:
                shutil.rmtree(server.default_path)
            except Exception as e:
                server.get_logger().error(e)
