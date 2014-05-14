#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import actionlib
from ros_datacentre_msgs.msg import  MoveEntriesAction, MoveEntriesGoal, StringList
import sys

def feedback(feedback):
    print feedback

if __name__ == '__main__':
    rospy.init_node("mongodb_replicator_client")

    client = actionlib.SimpleActionClient('move_datacentre_entries', MoveEntriesAction)
    client.wait_for_server()

    collections = sys.argv[1:]

    print collections

    # but this is the default anyway
    twenty_four_hrs_ago = rospy.Duration(60 * 60 * 24)
    goal = MoveEntriesGoal(collections=StringList(collections), move_before=twenty_four_hrs_ago, delete_after_move=True)


    client.send_goal(goal, feedback_cb=feedback)
    client.wait_for_result()