# -*- coding: utf-8 -*-
"""AR test-target task executor: recognize, align, and simulated-shoot.

Called by MissionRunner after a task waypoint's navigation has SUCCEEDED.
One call performs, in order:

  1. stop and confirm the chassis has settled,
  2. rotate in place to find the target and align to it,
  3. stop again and confirm settled,
  4. fire a single simulated shot.

During navigation this module publishes nothing; it only writes /cmd_vel while
a task is actually executing, and always publishes a finite number of zero
velocity commands when it finishes. AR message imports are kept in this task
module so the pure-navigation entry points never need ar_track_alvar.

Detection is handled conservatively: markers are filtered by the requested
target id, their per-marker header (stamp + frame_id) is checked, coordinates
must be finite with z > 0, and both the image-timestamp age and the actual
wall-clock age since the message was received are limited. A paused /clock
therefore invalidates detections even though simulated time stops advancing.

Python 2.7 / Python 3 compatible.
"""
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers

from aigc_navigation.navigation_client import wall_time, is_shutdown
from aigc_navigation.sim_shooter import SimShooter

TASK_SUCCESS = 'success'
TASK_FAILED = 'failed'
TASK_TIMEOUT = 'timeout'
TASK_INTERRUPTED = 'interrupted'

_REASON_BY_STATUS = {
    TASK_TIMEOUT: 'timeout',
    TASK_INTERRUPTED: 'interrupted',
}


def _is_finite(v):
    return isinstance(v, (int, float)) and not math.isnan(v) and \
        not math.isinf(v)


class TaskExecutor(object):
    """Recognize/align/shoot for one target id, with no scheduling framework."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._shooter = SimShooter(cfg['shooter'])

        # Detection state is updated ONLY in callbacks; the control loop reads
        # it and never turns/shoots from inside a callback.
        self._markers = None
        self._markers_stamp = None
        self._odom = None
        self._odom_stamp = None

        self._subs = [
            rospy.Subscriber(cfg['ar_marker_topic'], AlvarMarkers,
                             self._on_markers),
            rospy.Subscriber(cfg['odom_topic'], Odometry, self._on_odom),
        ]

    # -- callbacks (data update only) ------------------------------------
    def _on_markers(self, msg):
        self._markers = msg
        self._markers_stamp = wall_time()

    def _on_odom(self, msg):
        self._odom = msg
        self._odom_stamp = wall_time()

    # -- helpers ----------------------------------------------------------
    def _publish_twist(self, linear_x, angular_z):
        t = Twist()
        t.linear.x = linear_x
        t.linear.y = 0.0
        t.angular.z = angular_z
        try:
            self._cmd_pub.publish(t)
        except Exception:
            pass

    def _stop(self):
        """Publish zero velocity a finite number of times."""
        for _ in range(self.cfg['stop_publish_count']):
            self._publish_twist(0.0, 0.0)
            time.sleep(self.cfg['stop_publish_interval'])

    def _interruptible_sleep(self, seconds):
        deadline = wall_time() + seconds
        while wall_time() < deadline:
            if is_shutdown():
                break
            time.sleep(0.01)

    def _image_age(self, stamp):
        try:
            return (rospy.get_rostime() - stamp).to_sec()
        except Exception:
            return None

    def _latest_valid_detection(self, target_id):
        """Return the single valid marker for target_id, or None.

        Returns None when: no message, the message is too old (wall clock),
        no marker matches target_id, multiple markers share the id (ambiguous,
        not blindly picked), the per-marker header has the wrong frame or an
        empty/too-old stamp, or the coordinates are non-finite / z <= 0.
        """
        msg = self._markers
        if msg is None or self._markers_stamp is None:
            return None
        # Actual receive age (wall clock): catches a paused /clock.
        if wall_time() - self._markers_stamp > self.cfg['receive_max_age']:
            return None

        candidates = []
        for marker in msg.markers:
            if marker.id != target_id:
                continue
            hdr = marker.header
            if hdr.frame_id != self.cfg['output_frame']:
                continue
            stamp = hdr.stamp
            age = self._image_age(stamp)
            if age is None or age < 0.0:
                continue
            if age > self.cfg['detection_max_age']:
                continue
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            z = marker.pose.pose.position.z
            if not (_is_finite(x) and _is_finite(y) and _is_finite(z)):
                continue
            if z <= 0.0:
                continue
            candidates.append(marker)

        if len(candidates) != 1:
            # No marker, or multiple same-id markers: do not pick blindly.
            return None
        return candidates[0]

    def _odom_stable(self):
        odom = self._odom
        if odom is None or self._odom_stamp is None:
            return False
        if wall_time() - self._odom_stamp > self.cfg['odom_max_age']:
            return False
        twist = odom.twist.twist
        lin = math.hypot(twist.linear.x, twist.linear.y)
        ang = abs(twist.angular.z)
        return (lin <= self.cfg['odom_max_linear'] and
                ang <= self.cfg['odom_max_angular'])

    def _stop_and_settle(self, timeout, deadline):
        """Stop and wait until odom is below thresholds, bounded by time."""
        self._publish_twist(0.0, 0.0)
        end = wall_time() + timeout
        while wall_time() < end and wall_time() < deadline:
            if is_shutdown():
                return False
            if self._odom_stable():
                return True
            self._publish_twist(0.0, 0.0)
            self._interruptible_sleep(0.05)
        return self._odom_stable()

    def _search_and_align(self, target_id, deadline):
        """Rotate in place to find and align to the target.

        Returns TASK_SUCCESS (aligned), TASK_TIMEOUT, or TASK_INTERRUPTED.
        Only in-place rotation is commanded: linear.x/linear.y stay zero.
        """
        kp = self.cfg['kp']
        max_rate = self.cfg['max_angular_rate']
        search_rate = self.cfg['search_rate']
        align_tol = self.cfg['align_tolerance']
        stable_frames = self.cfg['stable_frames']
        period = 1.0 / self.cfg['control_rate']

        stable = 0
        last_stamp = None

        while wall_time() < deadline:
            if is_shutdown():
                self._stop()
                return TASK_INTERRUPTED

            det = self._latest_valid_detection(target_id)
            if det is None:
                # No valid target: clear progress and search in place.
                stable = 0
                last_stamp = None
                self._publish_twist(0.0, search_rate)
            else:
                x = det.pose.pose.position.x
                z = det.pose.pose.position.z
                error = math.atan2(x, z)
                if abs(error) <= align_tol:
                    # Aligned this frame. Count only a NEW image timestamp so
                    # a 20 Hz loop cannot double-count one 15 Hz detection.
                    if det.header.stamp != last_stamp:
                        stable += 1
                        last_stamp = det.header.stamp
                    self._publish_twist(0.0, 0.0)
                    if stable >= stable_frames:
                        return TASK_SUCCESS
                else:
                    # Not aligned: clear progress and steer proportionally.
                    stable = 0
                    last_stamp = None
                    cmd = -kp * error
                    if cmd > max_rate:
                        cmd = max_rate
                    elif cmd < -max_rate:
                        cmd = -max_rate
                    self._publish_twist(0.0, cmd)

            self._interruptible_sleep(period)

        self._stop()
        return TASK_TIMEOUT

    # -- result -----------------------------------------------------------
    @staticmethod
    def _result(status, aligned, shot, reason):
        return {'status': status, 'aligned': aligned, 'shot': shot,
                'reason': reason}

    # -- entry point ------------------------------------------------------
    def execute(self, run_id, waypoint, target_id, allow_shot=True):
        """Run the whole task for one waypoint and return a result dict.

        ``allow_shot`` enforces "at most one simulated shot per waypoint per
        run": when False the task still aligns but does not fire again.
        """
        # Reset this task's caches and state; never reuse a previous task's.
        self._markers = None
        self._markers_stamp = None
        self._odom = None
        self._odom_stamp = None

        start = wall_time()
        deadline = start + self.cfg['task_timeout']

        # 1) stop and settle before searching.
        if not self._stop_and_settle(self.cfg['settle_timeout'], deadline):
            self._stop()
            return self._result(TASK_FAILED, False, None, 'did not settle')

        # 2) search + align.
        status = self._search_and_align(target_id, deadline)
        if status != TASK_SUCCESS:
            return self._result(status, False, None,
                                _REASON_BY_STATUS.get(status, ''))

        # 3) stop and settle again before shooting.
        if not self._stop_and_settle(self.cfg['settle_timeout'], deadline):
            self._stop()
            return self._result(TASK_FAILED, True, None,
                                'did not settle before shoot')

        # 4) fire once (unless this waypoint already fired this run).
        if not allow_shot:
            self._stop()
            return self._result(TASK_SUCCESS, True, None,
                                'already shot; no repeat')

        shot = self._shooter.shoot(run_id, waypoint, target_id,
                                   '%s_%s' % (run_id, waypoint))
        if is_shutdown():
            self._stop()
            return self._result(TASK_INTERRUPTED, True, shot,
                                'shutdown during shoot')
        if not shot['success']:
            self._stop()
            return self._result(TASK_FAILED, True, shot, shot['reason'])
        self._stop()
        return self._result(TASK_SUCCESS, True, shot, '')

    def cleanup(self):
        self._stop()
        self._shooter.cleanup()
