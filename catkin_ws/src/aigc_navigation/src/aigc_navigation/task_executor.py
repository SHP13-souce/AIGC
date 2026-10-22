# -*- coding: utf-8 -*-
"""AR test-target recognition, alignment and simulated shooting (round 4/5).

The executor subscribes to AR markers and odometry, filters a single target by
id/frame/freshness, turns in place until the bearing error is within tolerance
for a number of *stable* (strictly-increasing image timestamp) frames while the
chassis is settled, then fires one simulated shot through :class:`SimShooter`.

Every stage shares one absolute wall-clock deadline derived from
``task_timeout``, so parking, searching, aligning, re-settling, the pulse and
the feedback wait are all bounded by the same whole-task deadline. A shutdown
anywhere maps to ``interrupted`` and a deadline pass to ``timeout``; neither is
ever reported as success.

Python 2.7 / Python 3 compatible (no f-strings, no keyword-only args).
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
    """Recognize + align + settle + shoot a single AR target."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._shooter = SimShooter(cfg['shooter'])

        self._markers = None
        self._markers_stamp = None
        self._odom = None
        self._odom_stamp = None
        self._task_start_ros = None

        # Per-round shot quota: which waypoints actually fired, and whether the
        # shot succeeded. Quota is reserved *before* the trigger so an exception
        # after publishing cannot cause a duplicate shot.
        self._fired = set()
        self._shot_success = {}

        self._subs = [
            rospy.Subscriber(cfg['ar_marker_topic'], AlvarMarkers,
                             self._on_markers),
            rospy.Subscriber(cfg['odom_topic'], Odometry, self._on_odom),
        ]

    # -- callbacks (state update only) -----------------------------------
    def _on_markers(self, msg):
        self._markers = msg
        self._markers_stamp = wall_time()

    def _on_odom(self, msg):
        self._odom = msg
        self._odom_stamp = wall_time()

    # -- low-level helpers ------------------------------------------------
    def _publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        try:
            self._cmd_pub.publish(twist)
        except Exception:
            pass

    def _stop(self):
        for _ in range(self.cfg['stop_publish_count']):
            self._publish_twist(0.0, 0.0)
            time.sleep(self.cfg['stop_publish_interval'])

    def _interruptible_sleep(self, seconds):
        end = wall_time() + seconds
        while wall_time() < end:
            if is_shutdown():
                break
            time.sleep(0.01)

    @staticmethod
    def _valid_stamp(stamp):
        try:
            return stamp.to_sec() > 0.0
        except Exception:
            return False

    def _stamp_age(self, stamp):
        try:
            return (rospy.get_rostime() - stamp).to_sec()
        except Exception:
            return None

    # -- detection / odometry freshness -----------------------------------
    def _latest_valid_detection(self, target_id):
        msg = self._markers
        if msg is None or self._markers_stamp is None:
            return None
        # Message must have been received recently (receive age).
        if wall_time() - self._markers_stamp > self.cfg['receive_max_age']:
            return None
        task_start = self._task_start_ros
        if task_start is None:
            return None

        candidates = []
        for marker in msg.markers:
            if marker.id != target_id:
                continue
            hdr = marker.header
            if hdr.frame_id != self.cfg['output_frame']:
                continue
            stamp = hdr.stamp
            # Marker's own header stamp must be a valid positive time.
            if not self._valid_stamp(stamp):
                continue
            # Image must have been captured *after* this task started.
            try:
                if stamp.to_sec() <= task_start.to_sec():
                    continue
            except Exception:
                continue
            # Image age (ROS clock) must be within bounds.
            age = self._stamp_age(stamp)
            if age is None or age < 0.0 or age > self.cfg['detection_max_age']:
                continue

            pos = marker.pose.pose.position
            if not (_is_finite(pos.x) and _is_finite(pos.y) and
                    _is_finite(pos.z)):
                continue
            if pos.z <= 0.0:
                continue
            candidates.append(marker)

        if len(candidates) != 1:
            return None
        return candidates[0]

    def _odom_stable(self):
        odom = self._odom
        if odom is None or self._odom_stamp is None:
            return False
        # Message receive age.
        if wall_time() - self._odom_stamp > self.cfg['odom_max_age']:
            return False
        # odom header stamp must be valid and fresh: a freshly-received message
        # carrying a stale header does NOT prove the robot stopped.
        try:
            hdr_stamp = odom.header.stamp
        except Exception:
            return False
        if not self._valid_stamp(hdr_stamp):
            return False
        age = self._stamp_age(hdr_stamp)
        if age is None or age < 0.0 or age > self.cfg['odom_max_age']:
            return False

        twist = odom.twist.twist
        lin = math.hypot(twist.linear.x, twist.linear.y)
        ang = abs(twist.angular.z)
        return (lin <= self.cfg['odom_max_linear'] and
                ang <= self.cfg['odom_max_angular'])

    # -- stages ------------------------------------------------------------
    def _stop_and_settle(self, timeout, deadline):
        """Stop and wait until odometry reports the robot settled.

        Returns 'ok', 'interrupted', 'timeout' or 'not_settled'. After a
        timeout/interrupt this must NOT fall back to success just because the
        last odom sample happened to look stable.
        """
        if is_shutdown():
            return 'interrupted'
        self._publish_twist(0.0, 0.0)
        end = wall_time() + timeout
        while wall_time() < end and wall_time() < deadline:
            if is_shutdown():
                return 'interrupted'
            if self._odom_stable():
                return 'ok'
            self._publish_twist(0.0, 0.0)
            self._interruptible_sleep(0.05)
        if is_shutdown():
            return 'interrupted'
        if wall_time() >= deadline:
            return 'timeout'
        return 'not_settled'

    def _search_align(self, target_id, deadline):
        kp = self.cfg['kp']
        max_rate = self.cfg['max_angular_rate']
        search_rate = self.cfg['search_rate']
        align_tol = self.cfg['align_tolerance']
        stable_frames = self.cfg['stable_frames']
        period = 1.0 / self.cfg['control_rate']

        stable = 0
        last_counted = None

        while wall_time() < deadline:
            if is_shutdown():
                self._stop()
                return TASK_INTERRUPTED

            det = self._latest_valid_detection(target_id)
            if det is None:
                # Real loss / stale / no valid target: clear progress, search.
                stable = 0
                last_counted = None
                self._publish_twist(0.0, search_rate)
            else:
                error = self._alignment_error(det)
                if abs(error) > align_tol:
                    # Drifted off target: clear progress, steer.
                    stable = 0
                    last_counted = None
                    cmd = -kp * error
                    if cmd > max_rate:
                        cmd = max_rate
                    elif cmd < -max_rate:
                        cmd = -max_rate
                    self._publish_twist(0.0, cmd)
                elif self._odom_stable():
                    stamp = det.header.stamp
                    # Only strictly-increasing NEW image timestamps count; a
                    # same-frame re-read and a backward frame neither count nor
                    # clear (A,B,A stays 2, not 3).
                    if last_counted is None or \
                            stamp.to_sec() > last_counted.to_sec():
                        stable += 1
                        last_counted = stamp
                    self._publish_twist(0.0, 0.0)
                    if stable >= stable_frames:
                        return TASK_SUCCESS
                else:
                    # Aligned but the chassis is moving / odom stale, invalid
                    # or missing: the run of stable frames is broken. Clear the
                    # count (but keep last_counted so a same-frame re-read and a
                    # backward frame still cannot double-count); the robot must
                    # re-accumulate stable_frames consecutive *new*
                    # aligned-and-settled frames before a shot.
                    stable = 0
                    self._publish_twist(0.0, 0.0)

            self._interruptible_sleep(period)

        self._stop()
        return TASK_TIMEOUT

    @staticmethod
    def _result(status, aligned, shot, reason):
        return {'status': status, 'aligned': aligned, 'shot': shot,
                'reason': reason}

    def _alignment_error(self, detection):
        return math.atan2(detection.pose.pose.position.x,
                          detection.pose.pose.position.z)

    def _final_gate_and_shoot(self, run_id, waypoint, target_id, allow_shot,
                              deadline):
        """Re-confirm every firing condition at the moment of shooting."""
        if is_shutdown():
            return self._result(TASK_INTERRUPTED, True, None,
                                'shutdown before shoot')
        if wall_time() >= deadline:
            return self._result(TASK_TIMEOUT, True, None,
                                'timeout before shoot')

        det = self._latest_valid_detection(target_id)
        if det is None:
            return self._result(TASK_FAILED, True, None,
                                'target no longer valid before shoot')
        error = self._alignment_error(det)
        if abs(error) > self.cfg['align_tolerance']:
            return self._result(TASK_FAILED, True, None,
                                'target moved before shoot')
        if not self._odom_stable():
            return self._result(TASK_FAILED, True, None,
                                'not settled before shoot')

        may_shoot = allow_shot and waypoint not in self._fired
        if not may_shoot:
            # Quota consumed: only a prior *success* may be reused. A prior
            # failure/timeout/interrupt is never rewritten into success.
            if self._shot_success.get(waypoint, False):
                return self._result(TASK_SUCCESS, True, None,
                                    'no shot; prior success reused')
            return self._result(TASK_FAILED, True, None,
                                'no shot quota and no prior success')

        # Reserve the quota BEFORE firing so an exception cannot double-fire.
        self._fired.add(waypoint)
        shot = self._shooter.shoot(run_id, waypoint, target_id,
                                   '%s_%s' % (run_id, waypoint), deadline)
        self._shot_success[waypoint] = (shot['status'] == 'success')

        if shot['status'] == 'interrupted':
            return self._result(TASK_INTERRUPTED, True, shot, shot['reason'])
        if shot['status'] == 'timeout':
            return self._result(TASK_TIMEOUT, True, shot, shot['reason'])
        if shot['status'] == 'failed':
            return self._result(TASK_FAILED, True, shot, shot['reason'])
        return self._result(TASK_SUCCESS, True, shot, '')

    # -- entry point --------------------------------------------------------
    def execute(self, run_id, waypoint, target_id, allow_shot=True):
        self._task_start_ros = rospy.get_rostime()
        # Never reuse a previous task's cached state.
        self._markers = None
        self._markers_stamp = None
        self._odom = None
        self._odom_stamp = None

        deadline = wall_time() + self.cfg['task_timeout']
        result = None
        try:
            settled = self._stop_and_settle(self.cfg['settle_timeout'],
                                            deadline)
            if settled == 'interrupted':
                result = self._result(TASK_INTERRUPTED, False, None,
                                      'interrupted while settling')
            elif settled == 'timeout':
                result = self._result(TASK_TIMEOUT, False, None,
                                      'timeout while settling')
            elif settled != 'ok':
                result = self._result(TASK_FAILED, False, None,
                                      'did not settle')
            else:
                status = self._search_align(target_id, deadline)
                if status != TASK_SUCCESS:
                    result = self._result(status, False, None,
                                          _REASON_BY_STATUS.get(status, ''))
                else:
                    result = self._final_gate_and_shoot(
                        run_id, waypoint, target_id, allow_shot, deadline)
        finally:
            # Always stop turning and end any started pulse.
            self._stop()
            self._shooter.stop_pulse()

        # A shutdown that lands during the cleanup above is still an
        # interrupt, and takes priority over a prior failure/timeout/skip.
        # Preserve any already-completed shot (and its feedback) rather than
        # wiping it: an interrupt must not erase a fired shot.
        if is_shutdown() and result is not None and \
                result['status'] != TASK_INTERRUPTED:
            result = self._result(TASK_INTERRUPTED,
                                  result.get('aligned', False),
                                  result.get('shot'),
                                  'interrupted during cleanup')
        return result

    def cleanup(self):
        self._stop()
        self._shooter.cleanup()
