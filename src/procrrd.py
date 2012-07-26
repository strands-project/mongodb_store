#!/usr/bin/python

###########################################################################
#  procrrd.py - create RRD graphs for single process using rrdtool
#
#  Created: Tue Jul 17 12:03:41 2012
#  Copyright  2012  Tim Niemueller [www.niemueller.de]
#                   RWTH Aachen University
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
import multiprocessing

from pymongo import Connection
import rrdtool

LOOPTIME  = 10
GRAPHTIME = 60

USER_HZ   = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
PAGE_SIZE = os.sysconf(os.sysconf_names['SC_PAGE_SIZE'])

_quit    = False
_conn    = None
_admindb = None
_datadb  = None

class ProcRRD(object):
    def __init__(self, graph_clear = False, pid = None, graph_name = ""):
        self.quit        = False
        self.graph_clear = graph_clear
        self.pid         = pid
        if graph_name == "":
            self.graph_name = "%d" % pid
        else:
            self.graph_name = graph_name

        self.init_rrd()

    def create_rrd(self, file, *data_sources):
        if not os.path.isfile(file) or self.graph_clear:
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
        self.create_rrd("proc_%s.rrd" % self.graph_name,
                        "DS:cpu:GAUGE:30:0:100",
                        "DS:mem:GAUGE:30:0:U",
                        "DS:io_in:COUNTER:30:0:U",
                        "DS:io_out:COUNTER:30:0:U")


    def graph_rrd(self):
        rrdtool.graph("proc_%s.png" % self.graph_name,
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag", "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=Process Info",
                      "--vertical-label", "CPU Usage (%)",
                      "--right-axis", "100000000:0",
                      "--right-axis-label", "Memory Usage (MB)",
                      "DEF:cpu=proc_%s.rrd:cpu:AVERAGE:step=10" % self.graph_name,
                      "DEF:mem=proc_%s.rrd:mem:AVERAGE:step=10" % self.graph_name,
                      "DEF:io_in=proc_%s.rrd:io_in:AVERAGE:step=10" % self.graph_name,
                      "DEF:io_out=proc_%s.rrd:io_out:AVERAGE:step=10" % self.graph_name,
                      "CDEF:memavgS=mem,100000000,/",
                      "LINE1:cpu#b72921:CPU Usage (%)",
                      "GPRINT:cpu:LAST: Current\\:%8.2lf %s",
                      "GPRINT:cpu:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:cpu:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:memavgS#0000ff:Mem Usage (MB)",
                      "GPRINT:mem:LAST: Current\\:%8.2lf %s",
                      "GPRINT:mem:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:mem:MAX:Maximum\\:%8.2lf %s\\n")

                      #"AREA:io_in#ccccff:I/O In",
                      #"GPRINT:io_in:LAST: Current\\:%8.2lf %s",
                      #"GPRINT:io_in:AVERAGE:Average\\:%8.2lf %s",
                      #"GPRINT:io_in:MAX:Maximum\\:%8.2lf %s\\n",
                      #"AREA:io_out#ffe6cc:I/O Out",
                      #"GPRINT:io_out:LAST: Current\\:%8.2lf %s",
                      #"GPRINT:io_out:AVERAGE:Average\\:%8.2lf %s",
                      #"GPRINT:io_out:MAX:Maximum\\:%8.2lf %s\\n",



    
    def update_rrd(self, initial=False):
        fpstat = open("/proc/%d/stat" % self.pid)
        pstat = fpstat.readline().split()
        fpstat.close()

        ftstat = open("/proc/stat")
        tstat = ftstat.readline().split()
        ftstat.close()

        fio = open("/proc/%d/io" % self.pid)
        io = dict((n,int(v)) for n,v in (a.split(': ') for a in fio.readlines() ) )
        fio.close()

        utime_now = int(pstat[13])
        stime_now = int(pstat[14])
        time_total_now = sum([int(a) for a in tstat[1:]])
        # we are interested in a "per core" value, hence multiply by number of CPUs
        time_total_now /= multiprocessing.cpu_count()

        if not initial and (time_total_now - self.time_total_last) > 0:
            user_util = (utime_now - self.utime_last) / (time_total_now - self.time_total_last);
            sys_util = (stime_now - self.stime_last) / (time_total_now - self.time_total_last);
            rss = int(pstat[23]) * PAGE_SIZE

            data = ((user_util + sys_util) * 100, rss, int(io['rchar']), int(io['wchar']))
            print("New data", data)

            rrdtool.update("proc_%s.rrd" % self.graph_name, "N:%f:%d:%d:%d" % data)

        self.utime_last = utime_now
        self.stime_last = stime_now
        self.time_total_last = time_total_now


    def run(self):
        looping_threshold  = timedelta(0, LOOPTIME,  0)
        graphing_threshold = timedelta(0, GRAPHTIME, 0)

        graphing_last = datetime.now()

        
        self.update_rrd(initial=True)
        
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
    parser.add_option("--pid", dest="pid",
                      help="PID to monitor", metavar="PID", type="int")
    parser.add_option("--graph-clear", dest="graph_clear", default=False,
                      action="store_true",
                      help="Remove existing RRD files.")
    parser.add_option("--graph-name", dest="graph_name", default="",
                      metavar="GRAPH_NAME",
                      help="Name of graph file (format proc_GRAPH_NAME.rrd)")
    (options, args) = parser.parse_args()

    procrrd = ProcRRD(graph_clear=options.graph_clear,
                      pid=options.pid, graph_name=options.graph_name)
    procrrd.run()


if __name__ == "__main__":
    main(sys.argv)
