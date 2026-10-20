#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Send a single navigation goal to move_base and report the action result.

Usage:
  rosrun aigc_navigation send_goal.py _x:=1.2 _y:=0.0 _yaw:=0.0

Only an ActionStatus SUCCEEDED result is treated as success. Timeouts are
measured against the WALL clock (not ROS /clock), so a paused or slow
simulation cannot stall the script indefinitely. The goal is always sent in
the "map" frame.

Exit codes:
  0    goal SUCCEEDED
  1    move_base action server not available within ~wait_timeout seconds
  2    goal did not finish within ~result_timeout seconds
  3    goal finished with a non-success status (aborted/rejected/...)
  130  interrupted / ROS shutdown (SIGINT / Ctrl+C or rospy.is_shutdown())

This script is Python 2.7 / Python 3 compatible (ROS1 Melodic defaults to
Python 2.7).
"""
import sys

import rospy
import actionlib

from aigc_navigation.navigation_client import (
    NavigationClient,
    is_shutdown,
    status_text,
)


def main():
    rospy.init_node('send_goal')

    x = rospy.get_param('~x', 1.2)
    y = rospy.get_param('~y', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)
    # Both timeouts are real (wall-clock) seconds, independent of /clock.
    wait_timeout = rospy.get_param('~wait_timeout', 10.0)
    result_timeout = rospy.get_param('~result_timeout', 60.0)

    client = NavigationClient('move_base')
    rospy.on_shutdown(client.cancel_and_stop)

    def shutdown_exit():
        # ROS shutdown is an interrupt, not a navigation failure or timeout.
        rospy.logwarn('ROS shutdown detected; cleaning up.')
        client.cancel_and_stop()
        sys.exit(130)

    # --- Wait for the action server --------------------------------------
    rospy.loginfo('Waiting for move_base action server (wall timeout %.1fs)...' %
                  wait_timeout)
    try:
        connected = client.wait_for_server(wait_timeout)
    except KeyboardInterrupt:
        rospy.logwarn('Interrupted while waiting for server; cleaning up.')
        client.cancel_and_stop()
        sys.exit(130)

    if is_shutdown():
        shutdown_exit()
    if not connected:
        rospy.logerr('move_base action server not available within %.1fs' %
                     wait_timeout)
        sys.exit(1)

    # --- Send the goal ----------------------------------------------------
    # A shutdown can arrive at any moment (SIGINT may not re-raise
    # KeyboardInterrupt), so re-check right before sending.
    if is_shutdown():
        shutdown_exit()

    rospy.loginfo('Sending goal: x=%.3f y=%.3f yaw=%.3f rad' % (x, y, yaw))
    client.send_goal(x, y, yaw)

    # --- Wait for the result ---------------------------------------------
    try:
        final_state = client.wait_for_result(result_timeout)
    except KeyboardInterrupt:
        rospy.logwarn('Interrupted; cancelling goal and stopping.')
        client.cancel_and_stop()
        sys.exit(130)

    if is_shutdown():
        shutdown_exit()
    if final_state is None:
        rospy.logerr('Goal not finished within %.1fs (wall clock); cancelling.' %
                     result_timeout)
        client.cancel_and_stop()
        sys.exit(2)

    if final_state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('Goal SUCCEEDED: x=%.3f y=%.3f yaw=%.3f rad' % (x, y, yaw))
        sys.exit(0)
    else:
        rospy.logerr('Goal finished with status %s' % status_text(final_state))
        client.cancel_and_stop()
        sys.exit(3)


if __name__ == '__main__':
    main()
