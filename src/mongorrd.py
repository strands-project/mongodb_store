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

    def graph_rrd(self):
        rrdtool.graph("opcounters.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag", "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Op Counters",
                      "DEF:insert=opcounters.rrd:insert:AVERAGE:step=10",
                      "DEF:query=opcounters.rrd:query:AVERAGE:step=10",
                      "DEF:update=opcounters.rrd:update:AVERAGE:step=10",
                      "DEF:delete=opcounters.rrd:delete:AVERAGE:step=10",
                      "DEF:getmore=opcounters.rrd:getmore:AVERAGE:step=10",
                      "DEF:command=opcounters.rrd:command:AVERAGE:step=10",
                      "LINE1:insert#FF7200:Inserts",
                      "GPRINT:insert:LAST: Current\\:%8.2lf %s",
                      "GPRINT:insert:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:insert:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:query#503001:Queries",
                      "GPRINT:query:LAST: Current\\:%8.2lf %s",
                      "GPRINT:query:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:query:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:update#EDAC00:Updates",
                      "GPRINT:update:LAST: Current\\:%8.2lf %s",
                      "GPRINT:update:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:update:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:delete#506101:Deletes",
                      "GPRINT:delete:LAST: Current\\:%8.2lf %s",
                      "GPRINT:delete:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:delete:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:getmore#0CCCCC:Getmores",
                      "GPRINT:getmore:LAST:Current\\:%8.2lf %s",
                      "GPRINT:getmore:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:getmore:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:command#53CA05:Commands",
                      "GPRINT:command:LAST:Current\\:%8.2lf %s",
                      "GPRINT:command:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:command:MAX:Maximum\\:%8.2lf %s\\n")

        rrdtool.graph("memory.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag",
                      "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Memory Usage",
                      "DEF:rawresident=memory.rrd:resident:AVERAGE:step=10",
                      "DEF:rawvirtual=memory.rrd:virtual:AVERAGE:step=10",
                      "DEF:rawmapped=memory.rrd:mapped:AVERAGE:step=10",
                      "CDEF:resident=rawresident,1048576,*",
                      "CDEF:virtual=rawvirtual,1048576,*",
                      "CDEF:mapped=rawmapped,1048576,*",
                      "AREA:virtual#3B7AD9:Virtual",
                      "GPRINT:virtual:LAST: Current\\:%8.2lf %s",
                      "GPRINT:virtual:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:virtual:MAX:Maximum\\:%8.2lf %s\\n",
                      "AREA:mapped#6FD1BF:Mapped",
                      "GPRINT:mapped:LAST:  Current\\:%8.2lf %s",
                      "GPRINT:mapped:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:mapped:MAX:Maximum\\:%8.2lf %s\\n",
                      "AREA:resident#0E6E5C:Resident",
                      "GPRINT:resident:LAST:Current\\:%8.2lf %s",
                      "GPRINT:resident:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:resident:MAX:Maximum\\:%8.2lf %s\\n")
        
        rrdtool.graph("dbstats_collindext.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag",
                      "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Stats (Collections, Indexes, Extents)",
                      "DEF:collections=dbstats.rrd:collections:AVERAGE:step=1",
                      "DEF:indexes=dbstats.rrd:indexes:AVERAGE:step=1",
                      "DEF:numExtents=dbstats.rrd:numExtents:AVERAGE:step=1",
                      "LINE1:collections#FF7200:Collections",
                      "GPRINT:collections:LAST:Current\\:%8.2lf %s",
                      "GPRINT:collections:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:collections:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:indexes#EDAC00:Indexes",
                      "GPRINT:indexes:LAST:    Current\\:%8.2lf %s",
                      "GPRINT:indexes:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:indexes:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:numExtents#506101:Extents",
                      "GPRINT:numExtents:LAST:    Current\\:%8.2lf %s",
                      "GPRINT:numExtents:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:numExtents:MAX:Maximum\\:%8.2lf %s\\n")

        rrdtool.graph("dbstats_objects.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag",
                      "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Objects",
                      "DEF:objects=dbstats.rrd:objects:AVERAGE:step=1",
                      "LINE1:objects#503001:Objects",
                      "GPRINT:objects:LAST: Current\\:%8.2lf %s",
                      "GPRINT:objects:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:objects:MAX:Maximum\\:%8.2lf %s\\n")

        rrdtool.graph("dbstats_sizes.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag",
                      "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Stats (Sizes)",
                      "DEF:avgObjSize=dbstats.rrd:avgObjSize:AVERAGE:step=1",
                      "DEF:dataSize=dbstats.rrd:dataSize:AVERAGE:step=1",
                      "DEF:storageSize=dbstats.rrd:storageSize:AVERAGE:step=1",
                      "DEF:indexSize=dbstats.rrd:indexSize:AVERAGE:step=1",
                      "DEF:fileSize=dbstats.rrd:fileSize:AVERAGE:step=1",
                      "LINE1:avgObjSize#FF7200:Avg Object",
                      "GPRINT:avgObjSize:LAST:Current\\:%8.2lf %s",
                      "GPRINT:avgObjSize:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:avgObjSize:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:dataSize#503001:Data",
                      "GPRINT:dataSize:LAST:      Current\\:%8.2lf %s",
                      "GPRINT:dataSize:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:dataSize:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:storageSize#EDAC00:Storage",
                      "GPRINT:storageSize:LAST:   Current\\:%8.2lf %s",
                      "GPRINT:storageSize:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:storageSize:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:indexSize#506101:Index",
                      "GPRINT:indexSize:LAST:     Current\\:%8.2lf %s",
                      "GPRINT:indexSize:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:indexSize:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:fileSize#0CCCCC:File",
                      "GPRINT:fileSize:LAST:      Current\\:%8.2lf %s",
                      "GPRINT:fileSize:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:fileSize:MAX:Maximum\\:%8.2lf %s\\n")

        rrdtool.graph("indexes.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag",
                      "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Indexes",
                      "DEF:accesses=indexes.rrd:accesses:AVERAGE:step=1",
                      "DEF:hits=indexes.rrd:hits:AVERAGE:step=1",
                      "DEF:misses=indexes.rrd:misses:AVERAGE:step=1",
                      "DEF:resets=indexes.rrd:resets:AVERAGE:step=1",
                      "LINE1:accesses#FF7200:Accesses",
                      "GPRINT:accesses:LAST:Current\\:%8.2lf %s",
                      "GPRINT:accesses:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:accesses:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:hits#503001:Hits",
                      "GPRINT:hits:LAST:    Current\\:%8.2lf %s",
                      "GPRINT:hits:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:hits:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:misses#EDAC00:Misses",
                      "GPRINT:misses:LAST:  Current\\:%8.2lf %s",
                      "GPRINT:misses:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:misses:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:resets#506101:Resets",
                      "GPRINT:resets:LAST:  Current\\:%8.2lf %s",
                      "GPRINT:resets:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:resets:MAX:Maximum\\:%8.2lf %s\\n")

    
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
        rrdtool.update("dbstats.rrd", "N:%d:%d:%f:%d:%d:%d:%d:%d:%d" % dbs)

    def run(self):
        looping_threshold  = timedelta(0, LOOPTIME,  0)
        graphing_threshold = timedelta(0, GRAPHTIME, 0)

        graphing_last = datetime.now()
        
        while not self.quit:
            started = datetime.now()

            self.update_rrd()

            if datetime.now() - graphing_last > graphing_threshold:
                print("Generating graphs")
                self.graph_rrd()
                graphing_last = datetime.now()

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
