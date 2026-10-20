#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Send a single navigation goal to move_base and report the action result.

Usage:
  rosrun aigc_navigation send_goal.py _x:=1.2 _y:=0.0 _yaw:=0.0

Only an ActionStatus SUCCEEDED result is treated as success; a timeout, abort
or rejection exits non-zero. The goal is always sent in the "map" frame.

This script is Python 2.7 / Python 3 compatible (ROS1 Melodic defaults to
Python 2.7).
"""
import math
import sys

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion


def yaw_to_quaternion(yaw):
    """Return a Quaternion from a yaw angle around Z."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def publish_stop(pub):
    """Publish zero velocity a few times to bring the robot to a stop."""
    zero = Twist()
    for _ in range(5):
        pub.publish(zero)
        rospy.sleep(0.05)


def status_text(state):
    names = {
        actionlib.GoalStatus.PENDING: 'PENDING',
        actionlib.GoalStatus.ACTIVE: 'ACTIVE',
        actionlib.GoalStatus.PREEMPTED: 'PREEMPTED',
        actionlib.GoalStatus.SUCCEEDED: 'SUCCEEDED',
        actionlib.GoalStatus.ABORTED: 'ABORTED',
        actionlib.GoalStatus.REJECTED: 'REJECTED',
        actionlib.GoalStatus.PREEMPTING: 'PREEMPTING',
        actionlib.GoalStatus.RECALLING: 'RECALLING',
        actionlib.GoalStatus.RECALLED: 'RECALLED',
        actionlib.GoalStatus.LOST: 'LOST',
    }
    return names.get(state, 'UNKNOWN(%d)' % state)


def main():
    rospy.init_node('send_goal')

    x = rospy.get_param('~x', 1.2)
    y = rospy.get_param('~y', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)
    wait_timeout = rospy.get_param('~wait_timeout', 10.0)
    result_timeout = rospy.get_param('~result_timeout', 60.0)

    stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def cancel_and_stop():
        # Cancel only this client's goal (never cancel_all_goals).
        try:
            client.cancel_goal()
        except Exception:
            pass
        publish_stop(stop_pub)

    rospy.on_shutdown(cancel_and_stop)

    rospy.loginfo('Waiting for move_base action server (timeout %.1fs)...',
                  wait_timeout)
    if not client.wait_for_server(rospy.Duration(wait_timeout)):
        rospy.logerr('move_base action server not available within %.1fs',
                     wait_timeout)
        sys.exit(1)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)

    rospy.loginfo('Sending goal: x=%.3f y=%.3f yaw=%.3f rad', x, y, yaw)
    client.send_goal(goal)

    finished = False
    try:
        finished = client.wait_for_result(rospy.Duration(result_timeout))
    except KeyboardInterrupt:
        rospy.logwarn('Interrupted; cancelling goal and stopping.')
        cancel_and_stop()
        sys.exit(130)

    if not finished:
        rospy.logerr('Goal not finished within %.1fs; cancelling.',
                     result_timeout)
        cancel_and_stop()
        sys.exit(1)

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('Goal SUCCEEDED: x=%.3f y=%.3f yaw=%.3f rad', x, y, yaw)
        sys.exit(0)
    else:
        rospy.logerr('Goal finished with status %s', status_text(state))
        cancel_and_stop()
        sys.exit(1)


if __name__ == '__main__':
    main()
