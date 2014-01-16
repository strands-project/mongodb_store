#!/bin/bash

WIDTH=560

rrdtool graph opcounters.png \
  --disable-rrdtool-tag \
  --start=-600 --end=-10 \
  --width=$WIDTH \
  --font LEGEND:10: --font UNIT:8: --font TITLE:12: --font AXIS:8: \
  --title="MongoDB Op Counters" \
  DEF:insert=opcounters.rrd:insert:AVERAGE:step=10 \
  DEF:query=opcounters.rrd:query:AVERAGE:step=10 \
  DEF:update=opcounters.rrd:update:AVERAGE:step=10 \
  DEF:delete=opcounters.rrd:delete:AVERAGE:step=10 \
  DEF:getmore=opcounters.rrd:getmore:AVERAGE:step=10 \
  DEF:command=opcounters.rrd:command:AVERAGE:step=10 \
  LINE1:insert#FF7200:Inserts \
    GPRINT:insert:LAST:" Current\:%8.2lf %s" \
    GPRINT:insert:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:insert:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:query#503001:Queries \
    GPRINT:query:LAST:" Current\:%8.2lf %s" \
    GPRINT:query:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:query:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:update#EDAC00:Updates \
    GPRINT:update:LAST:" Current\:%8.2lf %s" \
    GPRINT:update:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:update:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:delete#506101:Deletes \
    GPRINT:delete:LAST:" Current\:%8.2lf %s" \
    GPRINT:delete:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:delete:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:getmore#0CCCCC:Getmores \
    GPRINT:getmore:LAST:"Current\:%8.2lf %s" \
    GPRINT:getmore:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:getmore:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:command#53CA05:Commands \
    GPRINT:command:LAST:"Current\:%8.2lf %s" \
    GPRINT:command:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:command:MAX:"Maximum\:%8.2lf %s\n" \

rrdtool graph memory.png \
  --disable-rrdtool-tag \
  --width=$WIDTH \
  --font LEGEND:10: --font UNIT:8: --font TITLE:12: --font AXIS:8: \
  --title="MongoDB Memory Usage" \
  DEF:rawresident=memory.rrd:resident:AVERAGE:step=10 \
  DEF:rawvirtual=memory.rrd:virtual:AVERAGE:step=10 \
  DEF:rawmapped=memory.rrd:mapped:AVERAGE:step=10 \
  CDEF:resident=rawresident,1048576,* \
  CDEF:virtual=rawvirtual,1048576,* \
  CDEF:mapped=rawmapped,1048576,* \
  AREA:virtual#3B7AD9:"Virtual" \
    GPRINT:virtual:LAST:" Current\:%8.2lf %s" \
    GPRINT:virtual:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:virtual:MAX:"Maximum\:%8.2lf %s\n" \
  AREA:mapped#6FD1BF:"Mapped" \
    GPRINT:mapped:LAST:"  Current\:%8.2lf %s" \
    GPRINT:mapped:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:mapped:MAX:"Maximum\:%8.2lf %s\n" \
  AREA:resident#0E6E5C:"Resident" \
    GPRINT:resident:LAST:"Current\:%8.2lf %s" \
    GPRINT:resident:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:resident:MAX:"Maximum\:%8.2lf %s\n"

rrdtool graph dbstats_collindext.png \
  --disable-rrdtool-tag\
  --width=$WIDTH \
  --font LEGEND:10: --font UNIT:8: --font TITLE:12: --font AXIS:8: \
  --title="MongoDB Stats (Collections, Indexes, Extents)" \
  DEF:collections=dbstats.rrd:collections:AVERAGE:step=1 \
  DEF:indexes=dbstats.rrd:indexes:AVERAGE:step=1 \
  DEF:numExtents=dbstats.rrd:numExtents:AVERAGE:step=1 \
  LINE1:collections#FF7200:Collections \
    GPRINT:collections:LAST:"Current\:%8.2lf %s" \
    GPRINT:collections:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:collections:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:indexes#EDAC00:Indexes \
    GPRINT:indexes:LAST:"    Current\:%8.2lf %s" \
    GPRINT:indexes:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:indexes:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:numExtents#506101:Extents \
    GPRINT:numExtents:LAST:"    Current\:%8.2lf %s" \
    GPRINT:numExtents:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:numExtents:MAX:"Maximum\:%8.2lf %s\n"

rrdtool graph dbstats_objects.png \
  --disable-rrdtool-tag \
  --width=$WIDTH \
  --font LEGEND:10: --font UNIT:8: --font TITLE:12: --font AXIS:8: \
  --title="MongoDB Objects" \
  DEF:objects=dbstats.rrd:objects:AVERAGE:step=1 \
  LINE1:objects#503001:Objects \
    GPRINT:objects:LAST:" Current\:%8.2lf %s" \
    GPRINT:objects:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:objects:MAX:"Maximum\:%8.2lf %s\n" \

rrdtool graph dbstats_sizes.png \
  --disable-rrdtool-tag\
  --width=$WIDTH \
  --font LEGEND:10: --font UNIT:8: --font TITLE:12: --font AXIS:8: \
  --title="MongoDB Stats (Sizes)" \
  DEF:avgObjSize=dbstats.rrd:avgObjSize:AVERAGE:step=1 \
  DEF:dataSize=dbstats.rrd:dataSize:AVERAGE:step=1 \
  DEF:storageSize=dbstats.rrd:storageSize:AVERAGE:step=1 \
  DEF:indexSize=dbstats.rrd:indexSize:AVERAGE:step=1 \
  DEF:fileSize=dbstats.rrd:fileSize:AVERAGE:step=1 \
  LINE1:avgObjSize#FF7200:"Avg Object" \
    GPRINT:avgObjSize:LAST:"Current\:%8.2lf %s" \
    GPRINT:avgObjSize:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:avgObjSize:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:dataSize#503001:"Data" \
    GPRINT:dataSize:LAST:"      Current\:%8.2lf %s" \
    GPRINT:dataSize:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:dataSize:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:storageSize#EDAC00:"Storage" \
    GPRINT:storageSize:LAST:"   Current\:%8.2lf %s" \
    GPRINT:storageSize:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:storageSize:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:indexSize#506101:"Index" \
    GPRINT:indexSize:LAST:"     Current\:%8.2lf %s" \
    GPRINT:indexSize:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:indexSize:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:fileSize#0CCCCC:"File" \
    GPRINT:fileSize:LAST:"      Current\:%8.2lf %s" \
    GPRINT:fileSize:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:fileSize:MAX:"Maximum\:%8.2lf %s\n"

rrdtool graph indexes.png \
  --disable-rrdtool-tag\
  --width=$WIDTH \
  --font LEGEND:10: --font UNIT:8: --font TITLE:12: --font AXIS:8: \
  --title="MongoDB Indexes" \
  DEF:accesses=indexes.rrd:accesses:AVERAGE:step=1 \
  DEF:hits=indexes.rrd:hits:AVERAGE:step=1 \
  DEF:misses=indexes.rrd:misses:AVERAGE:step=1 \
  DEF:resets=indexes.rrd:resets:AVERAGE:step=1 \
  LINE1:accesses#FF7200:"Accesses" \
    GPRINT:accesses:LAST:"Current\:%8.2lf %s" \
    GPRINT:accesses:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:accesses:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:hits#503001:"Hits" \
    GPRINT:hits:LAST:"    Current\:%8.2lf %s" \
    GPRINT:hits:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:hits:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:misses#EDAC00:"Misses" \
    GPRINT:misses:LAST:"  Current\:%8.2lf %s" \
    GPRINT:misses:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:misses:MAX:"Maximum\:%8.2lf %s\n" \
  LINE1:resets#506101:"Resets" \
    GPRINT:resets:LAST:"  Current\:%8.2lf %s" \
    GPRINT:resets:AVERAGE:"Average\:%8.2lf %s" \
    GPRINT:resets:MAX:"Maximum\:%8.2lf %s\n" \

eog indexes.png dbstats_sizes.png dbstats_collindext.png dbstats_objects.png memory.png opcounters.png

