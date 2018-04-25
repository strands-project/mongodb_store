#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import argparse
from bson import json_util
import rospy
import actionlib
from mongodb_store_msgs.msg import StringList, StringPair, StringPairList
from mongodb_store_msgs.msg import MoveEntriesAction, MoveEntriesGoal
from mongodb_store_msgs.srv import MongoQueryMsgRequest


def feedback(feedback):
    rospy.loginfo(feedback)


def parse_args(args):
    p = argparse.ArgumentParser()
    p.add_argument('database', type=str,
                   help='The db to move entries from')
    p.add_argument('collection', type=str, nargs='+',
                   help='The collections to move entries from')
    p.add_argument('--move-before', type=int, default=60*60*24,  # 24 hrs
                   help='Only entries before rospy.Time.now() - move_before are moved. if 0, all are moved')
    p.add_argument('--delete-after-move', action='store_true',
                   help='Delete moved entries after replication')
    p.add_argument('--query', '-q', type=str, default='{}',
                   help='Only entries that are matched by the query are moved.')
    return p.parse_args(args)


if __name__ == '__main__':
    rospy.init_node("mongodb_replicator_client")

    args = parse_args(rospy.myargv()[1:])

    # validate parameters
    try:
        data = json_util.loads(args.query)
        query = StringPairList(
            pairs=[StringPair(first=MongoQueryMsgRequest.JSON_QUERY,
                              second=json_util.dumps(data))])
    except:
        raise ValueError('The query is invalid')
    if args.move_before < 0:
        raise ValueError('move_before time must be >= 0')
    move_before = rospy.Duration(args.move_before)
    goal = MoveEntriesGoal(
        database=args.database,
        collections=StringList(args.collection),
        move_before=move_before,
        delete_after_move=args.delete_after_move,
        query=query)

    rospy.loginfo('Moves entries from (db: %s, cols: %s)' % (args.database, args.collection))
    rospy.loginfo('before time: %s' % (rospy.Time.now() - move_before))
    rospy.loginfo('delete after move: %s' % args.delete_after_move)
    rospy.loginfo('with query: %s' % args.query)

    client = actionlib.SimpleActionClient('move_mongodb_entries', MoveEntriesAction)
    client.wait_for_server()

    client.send_goal(goal, feedback_cb=feedback)
    client.wait_for_result()
