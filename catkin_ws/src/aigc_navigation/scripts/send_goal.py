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
  130  interrupted (SIGINT / Ctrl+C)

This script is Python 2.7 / Python 3 compatible (ROS1 Melodic defaults to
Python 2.7).
"""
import math
import sys
import threading
import time

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion

# Wall-clock source with a Python 2.7 fallback (time.monotonic is Python 3.3+).
try:
    _wall_clock = time.monotonic
except AttributeError:
    _wall_clock = time.time

# GoalStatus states after which the goal can no longer make progress.
_TERMINAL = (
    actionlib.GoalStatus.SUCCEEDED,
    actionlib.GoalStatus.ABORTED,
    actionlib.GoalStatus.REJECTED,
    actionlib.GoalStatus.PREEMPTED,
    actionlib.GoalStatus.RECALLED,
    actionlib.GoalStatus.LOST,
)


def wall_time():
    return _wall_clock()


def yaw_to_quaternion(yaw):
    """Return a Quaternion from a yaw angle around Z."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


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


def publish_stop(pub):
    """Publish zero velocity a few times, using wall-clock sleeps."""
    zero = Twist()
    for _ in range(5):
        try:
            pub.publish(zero)
        except Exception:
            break
        time.sleep(0.05)


def wait_for_server_wall(client, timeout):
    """Wait for the action server, bounded by wall-clock ``timeout`` seconds.

    The blocking connection wait runs in a daemon thread so a frozen /clock
    (e.g. a paused simulation) cannot stall the deadline. The thread only
    waits for the connection; it never sends the goal. If the deadline passes
    first, the main program returns False and can exit (the daemon thread is
    abandoned).
    """
    connected = {'ok': False}

    def _connect():
        try:
            client.wait_for_server()  # blocks until the server appears
            connected['ok'] = True
        except Exception:
            pass

    thread = threading.Thread(target=_connect)
    thread.daemon = True  # set as attribute for Python 2.7 compatibility
    thread.start()

    deadline = wall_time() + timeout
    while wall_time() < deadline and not connected['ok']:
        if rospy.is_shutdown():
            break
        time.sleep(0.1)
    return connected['ok']


def wait_for_result_wall(client, timeout):
    """Poll this client's action state, bounded by wall-clock ``timeout``.

    Returns the terminal GoalStatus, or None if the deadline passes first.
    """
    deadline = wall_time() + timeout
    while wall_time() < deadline:
        if rospy.is_shutdown():
            break
        try:
            state = client.get_state()
        except Exception:
            time.sleep(0.1)
            continue
        if state in _TERMINAL:
            return state
        time.sleep(0.1)
    return None


def main():
    rospy.init_node('send_goal')

    x = rospy.get_param('~x', 1.2)
    y = rospy.get_param('~y', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)
    # Both timeouts are real (wall-clock) seconds, independent of /clock.
    wait_timeout = rospy.get_param('~wait_timeout', 10.0)
    result_timeout = rospy.get_param('~result_timeout', 60.0)

    stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    goal_state = {'sent': False}

    def cancel_and_stop():
        # Re-entrant cleanup: each step is isolated so one failure cannot stop
        # the rest. Only cancel a goal this client actually sent that has not
        # yet reached a terminal state; never cancel_all_goals.
        if goal_state['sent']:
            try:
                if client.get_state() not in _TERMINAL:
                    client.cancel_goal()
            except Exception:
                pass
        try:
            publish_stop(stop_pub)
        except Exception:
            pass

    rospy.on_shutdown(cancel_and_stop)

    rospy.loginfo('Waiting for move_base action server (wall timeout %.1fs)...',
                  wait_timeout)
    if not wait_for_server_wall(client, wait_timeout):
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
    goal_state['sent'] = True

    try:
        final_state = wait_for_result_wall(client, result_timeout)
    except KeyboardInterrupt:
        rospy.logwarn('Interrupted; cancelling goal and stopping.')
        cancel_and_stop()
        sys.exit(130)

    if final_state is None:
        rospy.logerr('Goal not finished within %.1fs (wall clock); cancelling.',
                     result_timeout)
        cancel_and_stop()
        sys.exit(2)

    if final_state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('Goal SUCCEEDED: x=%.3f y=%.3f yaw=%.3f rad', x, y, yaw)
        sys.exit(0)
    else:
        rospy.logerr('Goal finished with status %s', status_text(final_state))
        cancel_and_stop()
        sys.exit(3)


if __name__ == '__main__':
    main()
