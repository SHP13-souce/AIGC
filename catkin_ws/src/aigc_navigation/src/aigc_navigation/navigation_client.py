# -*- coding: utf-8 -*-
"""Shared move_base navigation client for send_goal.py and mission_manager.py.

This module must NOT call rospy.init_node() or sys.exit(); each entry script
owns its node and its exit codes. All waits use the wall clock (time.monotonic
with a Python 2.7 fallback) so a paused /clock cannot stall them.

Python 2.7 / Python 3 compatible.
"""
import math
import threading
import time

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion

try:
    _wall_clock = time.monotonic
except AttributeError:
    _wall_clock = time.time

# GoalStatus states after which a goal can no longer make progress.
TERMINAL_STATES = (
    actionlib.GoalStatus.SUCCEEDED,
    actionlib.GoalStatus.ABORTED,
    actionlib.GoalStatus.REJECTED,
    actionlib.GoalStatus.PREEMPTED,
    actionlib.GoalStatus.RECALLED,
    actionlib.GoalStatus.LOST,
)

# Definitive terminal states. LOST is excluded: when the client is LOST it does
# not know whether the goal actually finished, so LOST is never proof that a
# cancel was acknowledged.
DEFINITIVE_TERMINAL_STATES = (
    actionlib.GoalStatus.SUCCEEDED,
    actionlib.GoalStatus.ABORTED,
    actionlib.GoalStatus.REJECTED,
    actionlib.GoalStatus.PREEMPTED,
    actionlib.GoalStatus.RECALLED,
)


def wall_time():
    """Wall-clock seconds (monotonic where available)."""
    return _wall_clock()


def is_shutdown():
    return rospy.is_shutdown()


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


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def publish_stop(pub, count=5, interval=0.05):
    """Publish zero velocity ``count`` times, using wall-clock sleeps."""
    zero = Twist()
    for _ in range(count):
        try:
            pub.publish(zero)
        except Exception:
            break
        time.sleep(interval)


def wait_for_server_wall(client, timeout):
    """Wait for the action server, bounded by wall-clock ``timeout`` seconds.

    Returns True only when the server actually connects. The blocking
    connection wait runs in a daemon thread so a frozen /clock cannot stall
    the deadline. If ROS shuts down while waiting, the loop breaks early and
    returns the (still False) result; the caller detects shutdown itself.
    """
    connected = {'ok': False}

    def _connect():
        try:
            connected['ok'] = bool(client.wait_for_server())
        except Exception:
            connected['ok'] = False

    thread = threading.Thread(target=_connect)
    thread.daemon = True  # attribute assignment for Python 2.7 compatibility
    thread.start()

    deadline = wall_time() + timeout
    while wall_time() < deadline and not connected['ok']:
        if rospy.is_shutdown():
            break
        time.sleep(0.1)
    return connected['ok']


def wait_for_result_wall(client, timeout):
    """Poll the action state, bounded by wall-clock ``timeout``.

    Returns the terminal GoalStatus, or None if the deadline passes first or
    ROS shuts down. The caller distinguishes those two cases via is_shutdown().
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
        if state in TERMINAL_STATES:
            return state
        time.sleep(0.1)
    return None


class NavigationClient(object):
    """A persistent move_base action client for sequential goal sending."""

    def __init__(self, name='move_base', cmd_vel_topic='/cmd_vel'):
        self._client = actionlib.SimpleActionClient(name, MoveBaseAction)
        self._stop_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._goal_sent = False

    # -- connection -------------------------------------------------------
    def wait_for_server(self, timeout):
        """Wall-clock bounded connection wait; True when connected."""
        return wait_for_server_wall(self._client, timeout)

    def wait_for_server_blocking(self):
        """Block until connected (True) or ROS shutdown (False)."""
        try:
            return bool(self._client.wait_for_server())
        except Exception:
            return False

    # -- goal lifecycle ---------------------------------------------------
    def send_goal(self, x, y, yaw, frame_id='map'):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)
        self._client.send_goal(goal)
        self._goal_sent = True

    def get_state(self):
        return self._client.get_state()

    def wait_for_result(self, timeout):
        """Wait wall-clock bounded for a terminal state (or None on timeout)."""
        state = wait_for_result_wall(self._client, timeout)
        if state in TERMINAL_STATES:
            self._goal_sent = False
        return state

    def cancel_goal(self):
        """Cancel only our own unfinished goal (never cancel_all_goals)."""
        if not self._goal_sent:
            return
        try:
            if self._client.get_state() not in TERMINAL_STATES:
                self._client.cancel_goal()
        except Exception:
            pass
        self._goal_sent = False

    def wait_for_cancel(self, timeout):
        """Wait wall-clock bounded for a definitive terminal state after cancel.

        Returns True on a definitive terminal state (SUCCEEDED/ABORTED/REJECTED/
        PREEMPTED/RECALLED); False on timeout, shutdown, or LOST (LOST is not
        accepted as proof the goal finished).
        """
        deadline = wall_time() + timeout
        while wall_time() < deadline:
            if rospy.is_shutdown():
                return False
            try:
                state = self._client.get_state()
            except Exception:
                time.sleep(0.1)
                continue
            if state in DEFINITIVE_TERMINAL_STATES:
                return True
            if state == actionlib.GoalStatus.LOST:
                return False
            time.sleep(0.1)
        return False

    def stop(self, count=5, interval=0.05):
        publish_stop(self._stop_pub, count, interval)

    def cancel_and_stop(self):
        """Re-entrant cleanup: cancel own unfinished goal, then stop."""
        self.cancel_goal()
        try:
            self.stop()
        except Exception:
            pass
