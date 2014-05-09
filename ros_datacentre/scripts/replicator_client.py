#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import actionlib
from ros_datacentre_msgs.msg import  MoveEntriesAction, MoveEntriesGoal
import sys

def feedback(feedback):
    print feedback

if __name__ == '__main__':
    rospy.init_node("mongodb_replicator_client")

    client = actionlib.SimpleActionClient('move_datacentre_entries', MoveEntriesAction)
    client.wait_for_server()

    collections = sys.argv[1:]

    print collections

    goal = MoveEntriesGoal(collections=collections)

    client.send_goal(goal, feedback_cb=feedback)
    client.wait_for_result()