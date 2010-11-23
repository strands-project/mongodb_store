#!/usr/bin/python

###########################################################################
#  mongorrd.py - create RRD graphs for MongoDB using rrdtool
#
#  Created: Tue Nov 23 09:29:37 2010
#  Copyright  2010  Tim Niemueller [www.niemueller.de]
#                   Carnegie Mellon University
#                   Intel Labs Pittsburgh
###########################################################################

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

# make sure we aren't using floor division
from __future__ import division, with_statement

import os
import sys
import random
from pprint import pprint
from optparse import OptionParser
from datetime import datetime, timedelta
from time import sleep

from pymongo import Connection
import rrdtool

LOOPTIME  = 10
GRAPHTIME = 60

_quit    = False
_conn    = None
_admindb = None
_datadb  = None

class MongoRRD(object):
    def __init__(self, mongodb_host=None, mongodb_port=None, mongodb_name="roslog"):
        self.quit     = False
        self._conn    = Connection(mongodb_host, mongodb_port)
        self._admindb = self._conn["admin"]
        self._datadb  = self._conn[mongodb_name]

        self.init_rrd()

    def create_rrd(self, file, *data_sources):
        rrdtool.create(file, "--step", "10", "--start", "0",
                       # remember that we always need to add the previous RRA time range
                       # hence number of rows is not directly calculated by desired time frame
                       "RRA:AVERAGE:0.5:1:720",    #  2 hours of 10 sec  averages
                       "RRA:AVERAGE:0.5:3:1680",   # 12 hours of 30 sec  averages
                       "RRA:AVERAGE:0.5:30:456",   #  1 day   of  5 min  averages
                       "RRA:AVERAGE:0.5:180:412",  #  7 days  of 30 min  averages
                       "RRA:AVERAGE:0.5:720:439",  #  4 weeks of  2 hour averages
                       "RRA:AVERAGE:0.5:8640:402", #  1 year  of  1 day averages
                       "RRA:MIN:0.5:1:720",
                       "RRA:MIN:0.5:3:1680",
                       "RRA:MIN:0.5:30:456",
                       "RRA:MIN:0.5:180:412",
                       "RRA:MIN:0.5:720:439",
                       "RRA:MIN:0.5:8640:402",
                       "RRA:MAX:0.5:1:720",
                       "RRA:MAX:0.5:3:1680",
                       "RRA:MAX:0.5:30:456",
                       "RRA:MAX:0.5:180:412",
                       "RRA:MAX:0.5:720:439",
                       "RRA:MAX:0.5:8640:402",
                       *data_sources)
        

    def init_rrd(self):
        self.create_rrd("opcounters.rrd",
                        "DS:insert:COUNTER:30:0:U",
                        "DS:query:COUNTER:30:0:U",
                        "DS:update:COUNTER:30:0:U",
                        "DS:delete:COUNTER:30:0:U",
                        "DS:getmore:COUNTER:30:0:U",
                        "DS:command:COUNTER:30:0:U")

        self.create_rrd("memory.rrd",
                        "DS:resident:GAUGE:30:0:U",
                        "DS:virtual:GAUGE:30:0:U",
                        "DS:mapped:GAUGE:30:0:U")
        
        self.create_rrd("indexes.rrd",
                        "DS:accesses:COUNTER:30:0:U",
                        "DS:hits:COUNTER:30:0:U",
                        "DS:misses:COUNTER:30:0:U",
                        "DS:resets:COUNTER:30:0:U")

        self.create_rrd("locks.rrd", "DS:locktime:COUNTER:30:0:U")

        self.create_rrd("dbstats.rrd",
                        "DS:collections:GAUGE:30:0:U",
                        "DS:objects:GAUGE:30:0:U",
                        "DS:avgObjSize:GAUGE:30:0:U",
                        "DS:dataSize:GAUGE:30:0:U",
                        "DS:storageSize:GAUGE:30:0:U",
                        "DS:numExtents:GAUGE:30:0:U",
                        "DS:indexes:GAUGE:30:0:U",
                        "DS:indexSize:GAUGE:30:0:U",
                        "DS:fileSize:GAUGE:30:0:U")

    def update_rrd(self):
        status = self._admindb.command("serverStatus")
        ocs = status["opcounters"]
        opcounters = (ocs["insert"], ocs["query"], ocs["update"], ocs["delete"], ocs["getmore"], ocs["command"])
        mem = status["mem"]
        meminfo = (mem["resident"], mem["virtual"], mem["mapped"])
        ind = status["indexCounters"]["btree"]
        indexinfo = (ind["accesses"], ind["hits"], ind["misses"], ind["resets"])
        lockinfo = status["globalLock"]["lockTime"]
        rrdtool.update("opcounters.rrd", "N:%d:%d:%d:%d:%d:%d" % opcounters)
        rrdtool.update("memory.rrd", "N:%d:%d:%d" % meminfo)
        rrdtool.update("indexes.rrd", "N:%d:%d:%d:%d" % indexinfo)
        rrdtool.update("locks.rrd", "N:%d" % lockinfo)

        dbstats = self._datadb.command("dbStats")
        dbs = (dbstats["collections"], dbstats["objects"], dbstats["avgObjSize"],
               dbstats["dataSize"], dbstats["storageSize"], dbstats["numExtents"],
               dbstats["indexes"], dbstats["indexSize"], dbstats["fileSize"])
        rrdtool.update("dbstats.rrd", "N:%d:%d:%d:%d:%d:%d:%d:%d:%d" % dbs)

    def graph_rrd(self):
        pass

    def run(self):
        looping_threshold  = timedelta(0, LOOPTIME,  0)
        graphing_threshold = timedelta(0, GRAPHTIME, 0)
        
        while not self.quit:
            started = datetime.now()

            self.update_rrd()

            # the following code makes sure we run once per second, taking
            # varying run-times and interrupted sleeps into account
            td = datetime.now() - started
            while td < looping_threshold:
                sleeptime = LOOPTIME - (td.microseconds + (td.seconds + td.days * 24 * 3600) * 10**6) / 10**6
                if sleeptime > 0:
                    sleep(sleeptime)
                td = datetime.now() - started

def main(argv):
    parser = OptionParser()
    parser.add_option("--mongodb-host", dest="mongodb_host",
                      help="Hostname of MongoDB", metavar="HOST",
                      default="localhost")
    parser.add_option("--mongodb-port", dest="mongodb_port",
                      help="Hostname of MongoDB", type="int",
                      metavar="PORT", default=27017)
    parser.add_option("--mongodb-name", dest="mongodb_name",
                      help="Name of DB in which to store values",
                      metavar="NAME", default="roslog")
    (options, args) = parser.parse_args()

    mongorrd = MongoRRD(mongodb_host=options.mongodb_host,
                        mongodb_port=options.mongodb_port,
                        mongodb_name=options.mongodb_name)
    mongorrd.run()


if __name__ == "__main__":
    main(sys.argv)
