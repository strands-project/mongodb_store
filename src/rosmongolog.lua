#!/usr/bin/lua

----------------------------------------------------------------------------
--  skillenv.lua - Skiller skill environment functions
--
--  Created: Wed Nov 03 17:34:31 2010
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

require("roslua")
roslua.assert_version(0,4,1)

roslua.init_node{master_uri=os.getenv("ROS_MASTER_URI"),
		 node_name="/rosmongolog"}

local dbname = "rosmongolog"
local dbhost = "localhost"
local topics = { {name = "/chatter", type = "std_msgs/String"} }

require("mongo")

local db = mongo.Connection.New()
db:connect("localhost")


for _, t in ipairs(topics) do
   local s = roslua.subscriber(t.name, t.type)
   local subcolname = t.name:gsub("/", "_")
   if subcolname:match("_.*") then subcolname = subcolname:sub(2) end
   local collection_name = dbname .. "." .. subcolname
   s:add_listener(function (message)
		     local doc = message:plain_value_table()
		     db:insert(collection_name, doc)
		  end)
   t.subscriber = s
end

roslua.run()
