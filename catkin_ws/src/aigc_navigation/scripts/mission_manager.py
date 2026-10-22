#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Run a sequential waypoint mission through move_base.

Flow:
  init / validate config
  -> wait for navigation readiness (action server + map -> base_footprint TF +
     simulation clock started), wall-clock bounded
  -> navigate to each waypoint in order, with finite retries
  -> on reaching a task waypoint, call the (stub) task interface
  -> reach the finish, save a per-run JSON record, and exit

Exit codes:
  0    all required waypoints succeeded and the finish was reached (COMPLETED)
  1    invalid configuration, or navigation not ready in time
  2    navigation failure, timeout, unconfirmed cancel, or lost connection
  3    completed with skipped task waypoints (PARTIAL)
  130  interrupted (SIGINT / Ctrl+C) or ROS shutdown

Python 2.7 / Python 3 compatible (ROS1 Melodic defaults to Python 2.7).
"""
import json
import math
import os
import sys
import time
import uuid

import rospy
import actionlib
import tf

from aigc_navigation.navigation_client import (
    NavigationClient,
    wall_time,
    is_shutdown,
    status_text,
)

STATUS_COMPLETED = 'COMPLETED'
STATUS_PARTIAL = 'PARTIAL'
STATUS_FAILED = 'FAILED'
STATUS_STOPPED = 'STOPPED'

WP_SUCCESS = 'success'
WP_FAILED = 'failed'
WP_SKIPPED = 'skipped'
WP_INTERRUPTED = 'interrupted'

# Non-ordinary outcomes surfaced from navigate_to(); each stops the whole run.
OUT_LOST = 'lost'
OUT_CANCEL_UNCONFIRMED = 'cancel_unconfirmed'
OUT_SHUTDOWN = 'shutdown'

# Task result statuses returned by aigc_navigation.task_executor.execute().
TASK_OK = 'success'
TASK_INTERRUPTED = 'interrupted'

_VALID_KINDS = ('task', 'transit', 'finish')
_VALID_POLICIES = ('stop', 'skip')

# Default task -> AR target-id mapping when a task waypoint has no explicit
# target_id. Test targets only, not the official competition targets.
DEFAULT_TARGET_IDS = {'task_1': 0, 'task_2': 1, 'task_3': 2}


def _resolve_target_id(wp):
    """Return the target id for a task waypoint, or None if unresolved."""
    tid = wp.get('target_id')
    if tid is None:
        tid = DEFAULT_TARGET_IDS.get(wp['name'])
    return tid


class MissionRunner(object):
    """Holds per-run state so every restart starts fresh."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.client = NavigationClient('move_base')
        self.tf_listener = tf.TransformListener()
        self.records = []
        self.skipped_any = False
        self.in_progress_wp = None
        self.in_progress_attempts = 0
        self.task_cfg = None
        self.task_executor = None
        self.competition = None
        self.shot_waypoints = set()
        self.run_id = 'aigc_%s_%s' % (
            time.strftime('%Y%m%dT%H%M%S'), uuid.uuid4().hex[:8])
        self.start_wall = wall_time()
        self.start_time_str = time.strftime('%Y-%m-%dT%H:%M:%S')

    # -- cleanup ----------------------------------------------------------
    def cleanup(self):
        # Re-entrant: cancel our own unfinished goal, then publish zero velocity.
        try:
            self.client.cancel_and_stop()
        except Exception:
            pass
        if self.task_executor is not None:
            try:
                self.task_executor.cleanup()
            except Exception:
                pass
        if self.competition is not None:
            self.competition.cleanup()

    # -- readiness --------------------------------------------------------
    def wait_ready(self):
        """Wait (wall-clock bounded) for the navigation stack to be ready.

        Returns True when all three hold: the action server is connected, the
        map -> base_footprint TF is available, and simulation time has started.
        Never blocks on ROS time: the server connection is wall-clock bounded
        by server_timeout, and the TF check uses non-blocking canTransform()
        polling so a paused /clock cannot stall readiness.
        """
        frame = self.cfg['frame_id']
        deadline = wall_time() + self.cfg['ready_timeout']

        # 1) Action server connection, bounded by server_timeout but never past
        #    the overall readiness deadline.
        remaining = deadline - wall_time()
        if remaining <= 0:
            return False
        connect_timeout = min(self.cfg['server_timeout'], remaining)
        if not self.client.wait_for_server(connect_timeout):
            rospy.logerr('move_base action server not available within %.1fs' %
                         connect_timeout)
            return False
        if is_shutdown():
            return False

        # 2) TF + simulation clock, polled with time.sleep and bounded by the
        #    remaining portion of the overall readiness deadline.
        tf_ok = False
        clock_ok = False
        while wall_time() < deadline:
            if is_shutdown():
                return False
            if not tf_ok:
                try:
                    if self.tf_listener.canTransform(
                            frame, 'base_footprint', rospy.Time(0)):
                        tf_ok = True
                except Exception:
                    pass
            if rospy.get_rostime() > rospy.Time(0):
                clock_ok = True
            if tf_ok and clock_ok:
                return True
            time.sleep(0.1)
        return False

    # -- navigation of a single waypoint ----------------------------------
    def navigate_to(self, wp):
        """Navigate to one waypoint with finite retries and timeout handling.

        Returns a dict {status, attempts, reason}. status is WP_SUCCESS,
        WP_FAILED, OUT_LOST, OUT_CANCEL_UNCONFIRMED or OUT_SHUTDOWN.
        """
        goal_timeout = self.cfg['goal_timeout']
        cancel_timeout = self.cfg['cancel_timeout']
        max_retries = self.cfg['max_retries']
        frame_id = self.cfg['frame_id']

        self.in_progress_wp = wp
        self.in_progress_attempts = 0

        attempts = 0
        while True:
            if is_shutdown():
                return {'status': OUT_SHUTDOWN, 'attempts': attempts,
                        'reason': 'shutdown'}

            if not self.client.send_goal(wp['x'], wp['y'], wp['yaw'], frame_id):
                # A previous goal is still owned and unconfirmed: never start a
                # new goal over it.
                return {'status': OUT_CANCEL_UNCONFIRMED, 'attempts': attempts,
                        'reason': 'previous goal not confirmed'}
            attempts += 1
            self.in_progress_attempts = attempts

            state = self.client.wait_for_result(goal_timeout)

            if is_shutdown():
                return {'status': OUT_SHUTDOWN, 'attempts': attempts,
                        'reason': 'shutdown'}

            if state is None:
                # Execution timed out: cancel, then require a definitive
                # confirmation before retrying or continuing.
                self.client.cancel_goal()
                confirmed = self.client.wait_for_cancel(cancel_timeout)
                if is_shutdown():
                    # Shutdown during cancel confirmation is an interrupt, not
                    # an ordinary cancel failure.
                    return {'status': OUT_SHUTDOWN, 'attempts': attempts,
                            'reason': 'shutdown during cancel confirmation'}
                if confirmed:
                    if attempts <= max_retries:
                        rospy.logwarn(
                            'Waypoint %s timed out (attempt %d); retrying.' %
                            (wp['name'], attempts))
                        continue
                    return {'status': WP_FAILED, 'attempts': attempts,
                            'reason': 'timeout'}
                return {'status': OUT_CANCEL_UNCONFIRMED, 'attempts': attempts,
                        'reason': 'cancel not confirmed'}

            if state == actionlib.GoalStatus.SUCCEEDED:
                return {'status': WP_SUCCESS, 'attempts': attempts,
                        'reason': ''}

            if state == actionlib.GoalStatus.LOST:
                self.client.cancel_goal()
                return {'status': OUT_LOST, 'attempts': attempts,
                        'reason': 'lost'}

            # Ordinary failure (ABORTED / REJECTED / PREEMPTED / RECALLED).
            if attempts <= max_retries:
                rospy.logwarn(
                    'Waypoint %s failed with %s (attempt %d); retrying.' %
                    (wp['name'], status_text(state), attempts))
                continue
            return {'status': WP_FAILED, 'attempts': attempts,
                    'reason': status_text(state)}

    # -- task interface ---------------------------------------------------
    def on_task_reached(self, wp):
        rospy.loginfo('Arrived at task point: %s' % wp['name'])
        rospy.loginfo(
            'Task interface pending: recognition, alignment and striking '
            'are not connected yet.')
        self._interruptible_sleep(self.cfg['dwell_time'])

    @staticmethod
    def _interruptible_sleep(seconds):
        deadline = wall_time() + seconds
        while wall_time() < deadline:
            if is_shutdown():
                break
            time.sleep(0.05)

    # -- task execution ---------------------------------------------------
    def _run_task(self, wp, target_id, allow_shot):
        """Run the AR task for one waypoint (lazy import of TaskExecutor)."""
        if self.competition is not None:
            return self.competition.execute_task(self.run_id, wp, allow_shot)
        if self.task_executor is None:
            # Lazy import keeps ar_track_alvar out of the pure-navigation path.
            from aigc_navigation.task_executor import TaskExecutor
            self.task_executor = TaskExecutor(self.task_cfg)
        return self.task_executor.execute(self.run_id, wp['name'], target_id,
                                          allow_shot)

    # -- record -----------------------------------------------------------
    def _record(self, wp, result, attempts, reason, navigation_result=None,
                target_id=None, task=None):
        rec = {
            'name': wp['name'],
            'kind': wp['kind'],
            'x': wp['x'],
            'y': wp['y'],
            'yaw': wp['yaw'],
            'attempts': attempts,
            'result': result,
            'reason': reason,
        }
        if 'target_source' in wp:
            rec['target_source'] = wp['target_source']
        if navigation_result is not None:
            rec['navigation_result'] = navigation_result
        if target_id is not None:
            rec['target_id'] = target_id
        if task is not None:
            rec['task'] = task
        self.records.append(rec)

    def record_interrupt(self, reason):
        """Record the in-progress waypoint (if any) as interrupted."""
        if self.in_progress_wp is not None:
            wp = self.in_progress_wp
            target_id = None
            if wp.get('kind') == 'task' and self.cfg.get('execute_tasks'):
                target_id = _resolve_target_id(wp)
            self._record(wp, WP_INTERRUPTED, self.in_progress_attempts, reason,
                         target_id=target_id)
            self.in_progress_wp = None

    # -- main flow --------------------------------------------------------
    def run(self):
        if not self.wait_ready():
            if is_shutdown():
                self.cleanup()
                return (130, STATUS_STOPPED, 'shutdown during readiness')
            rospy.logerr('Navigation not ready within %.1fs' %
                         self.cfg['ready_timeout'])
            self.cleanup()
            return (1, STATUS_FAILED, 'navigation not ready')

        execute_tasks = bool(self.cfg.get('execute_tasks'))

        if (isinstance(self.cfg.get('competition'), dict) and
                self.cfg['competition'].get('enabled')):
            from aigc_navigation.competition import CompetitionIntegration
            self.competition = CompetitionIntegration(self.cfg['competition'],
                                                       self.task_cfg)
            prepared = self.competition.prepare(self.cfg['waypoints'])
            if prepared['status'] != TASK_OK:
                self.cleanup()
                if prepared['status'] == TASK_INTERRUPTED:
                    return (130, STATUS_STOPPED, prepared['reason'])
                return (2, STATUS_FAILED, prepared['reason'])

        for wp in self.cfg['waypoints']:
            if is_shutdown():
                self.cleanup()
                return (130, STATUS_STOPPED, 'shutdown')

            rospy.loginfo('Navigating to %s (%s): x=%.3f y=%.3f yaw=%.3f' %
                          (wp['name'], wp['kind'], wp['x'], wp['y'], wp['yaw']))
            res = self.navigate_to(wp)
            # navigate_to returned: the outcome is recorded below, so clear the
            # in-progress marker (it only matters if interrupted mid-nav).
            self.in_progress_wp = None

            if res['status'] == OUT_SHUTDOWN:
                self._record(wp, WP_INTERRUPTED, res['attempts'], res['reason'])
                rospy.logwarn('Waypoint %s: %s; stopping the run.' %
                              (wp['name'], res['reason']))
                self.cleanup()
                return (130, STATUS_STOPPED, res['reason'])

            if res['status'] == OUT_LOST:
                self._record(wp, WP_FAILED, res['attempts'], 'lost',
                             navigation_result='failed')
                rospy.logerr('Waypoint %s: LOST (connection unclear); '
                             'stopping the run.' % wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'lost connection')

            if res['status'] == OUT_CANCEL_UNCONFIRMED:
                self._record(wp, WP_FAILED, res['attempts'], res['reason'],
                             navigation_result='failed')
                rospy.logerr('Waypoint %s: %s; stopping the run.' %
                             (wp['name'], res['reason']))
                self.cleanup()
                return (2, STATUS_FAILED, res['reason'])

            if res['status'] == WP_SUCCESS:
                if wp['kind'] == 'task' and execute_tasks:
                    stop = self._run_task_waypoint(wp, res['attempts'])
                    if stop is not None:
                        return stop
                    continue

                if wp['kind'] == 'finish' and self.competition is not None:
                    spoken = self.competition.announce_finish()
                    if spoken['status'] != TASK_OK:
                        interrupted = spoken['status'] == TASK_INTERRUPTED
                        self._record(wp, WP_INTERRUPTED if interrupted else
                                     WP_FAILED, res['attempts'], spoken['reason'],
                                     navigation_result='success')
                        self.cleanup()
                        return (130 if interrupted else 2,
                                STATUS_STOPPED if interrupted else STATUS_FAILED,
                                spoken['reason'])

                self._record(wp, WP_SUCCESS, res['attempts'], '',
                             navigation_result='success')
                rospy.loginfo('Waypoint %s reached (attempts=%d).' %
                              (wp['name'], res['attempts']))
                if wp['kind'] == 'task':
                    self.on_task_reached(wp)
                continue

            # WP_FAILED: ordinary navigation failure after retries exhausted.
            kind = wp['kind']
            if kind == 'transit':
                self._record(wp, WP_FAILED, res['attempts'], res['reason'],
                             navigation_result='failed')
                rospy.logerr('Transit waypoint %s failed; stopping the run.' %
                             wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'transit failed')
            if kind == 'finish':
                self._record(wp, WP_FAILED, res['attempts'], res['reason'],
                             navigation_result='failed')
                rospy.logerr('Finish waypoint %s failed; run NOT completed.' %
                             wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'finish failed')
            if self.cfg['failure_policy'] == 'stop':
                self._record(wp, WP_FAILED, res['attempts'], res['reason'],
                             navigation_result='failed')
                rospy.logerr('Task waypoint %s failed; stopping '
                             '(failure_policy=stop).' % wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'task failed (stop)')
            # task + skip policy
            self._record(wp, WP_SKIPPED, res['attempts'],
                         'skipped after failure', navigation_result='failed')
            self.skipped_any = True
            rospy.logwarn('Task waypoint %s failed; skipping '
                          '(failure_policy=skip).' % wp['name'])

        if self.skipped_any:
            self.cleanup()
            if self.competition is not None and is_shutdown():
                return (130, STATUS_STOPPED, 'shutdown during mission cleanup')
            return (3, STATUS_PARTIAL, 'skipped task waypoints')
        self.cleanup()
        if self.competition is not None and is_shutdown():
            return (130, STATUS_STOPPED, 'shutdown during mission cleanup')
        return (0, STATUS_COMPLETED, '')

    def _run_task_waypoint(self, wp, attempts):
        """Run the AR task after a task waypoint's navigation succeeded.

        Returns a (exit_code, status, reason) tuple to stop the run, or None
        to continue to the next waypoint. Records navigation_result, task
        result, alignment and the simulated shot for the waypoint.
        """
        target_id = _resolve_target_id(wp)
        allow_shot = wp['name'] not in self.shot_waypoints
        self.in_progress_wp = wp
        self.in_progress_attempts = attempts
        task = self._run_task(wp, target_id, allow_shot)
        self.in_progress_wp = None
        if task['shot'] is not None:
            self.shot_waypoints.add(wp['name'])

        if task['status'] == TASK_INTERRUPTED:
            self._record(wp, WP_INTERRUPTED, attempts, task['reason'],
                         navigation_result='success', target_id=target_id,
                         task=task)
            rospy.logwarn('Waypoint %s task interrupted; stopping.' % wp['name'])
            self.cleanup()
            return (130, STATUS_STOPPED, task['reason'])

        if task['status'] == TASK_OK:
            self._record(wp, WP_SUCCESS, attempts, '',
                         navigation_result='success', target_id=target_id,
                         task=task)
            rospy.loginfo('Waypoint %s task succeeded (target %d).' %
                          (wp['name'], target_id))
            return None

        # Task failed / timed out: apply the failure policy.
        if self.cfg['failure_policy'] == 'stop':
            self._record(wp, WP_FAILED, attempts, task['reason'],
                         navigation_result='success', target_id=target_id,
                         task=task)
            rospy.logerr('Task %s failed (%s); stopping '
                         '(failure_policy=stop).' % (wp['name'], task['reason']))
            self.cleanup()
            return (2, STATUS_FAILED, task['reason'])

        self._record(wp, WP_SKIPPED, attempts, task['reason'],
                     navigation_result='success', target_id=target_id,
                     task=task)
        self.skipped_any = True
        rospy.logwarn('Task %s failed (%s); skipping '
                      '(failure_policy=skip).' % (wp['name'], task['reason']))
        return None

    # -- save the run record ---------------------------------------------
    def save_record(self, exit_code, final_status, reason):
        record = {
            'run_id': self.run_id,
            'start_time': self.start_time_str,
            'end_time': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'status': final_status,
            'exit_code': exit_code,
            'mission_mode': 'task' if self.cfg.get('execute_tasks')
                            else 'navigation',
            'reason': reason,
            'duration_sec': round(wall_time() - self.start_wall, 3),
            'waypoints': self.records,
        }
        if (isinstance(self.cfg.get('competition'), dict) and
                self.cfg['competition'].get('enabled')):
            record['mission_mode'] = 'competition_dry_run'
            record['integration'] = (self.competition.record if
                                     self.competition is not None else {})
        log_dir = self.cfg['log_dir']
        try:
            if not os.path.isdir(log_dir):
                os.makedirs(log_dir)
        except OSError as e:
            rospy.logerr('Cannot create log directory %s: %s' % (log_dir, e))
        path = os.path.join(log_dir, self.run_id + '.json')
        try:
            with open(path, 'w') as fh:
                json.dump(record, fh, indent=2, sort_keys=True)
            rospy.loginfo('Run record saved to %s' % path)
        except Exception as e:
            rospy.logerr('Failed to save run record to %s: %s' % (path, e))


def load_config():
    """Read mission parameters from the node private namespace."""
    return {
        'frame_id': rospy.get_param('~frame_id', 'map'),
        'ready_timeout': rospy.get_param('~ready_timeout', 30.0),
        'server_timeout': rospy.get_param('~server_timeout', 10.0),
        'goal_timeout': rospy.get_param('~goal_timeout', 60.0),
        'cancel_timeout': rospy.get_param('~cancel_timeout', 3.0),
        'max_retries': rospy.get_param('~max_retries', 1),
        'failure_policy': rospy.get_param('~failure_policy', 'stop'),
        'dwell_time': rospy.get_param('~dwell_time', 0.5),
        'execute_tasks': rospy.get_param('~execute_tasks', False),
        'waypoints': rospy.get_param('~waypoints', []),
        'log_dir': rospy.get_param('~log_dir', ''),
        'competition': rospy.get_param('~competition', {}),
    }


def load_task_config():
    """Read AR task parameters from the node's ~task private namespace.

    Called only when execute_tasks is enabled; every value has a default so
    this never fails even if task.yaml was not loaded.
    """
    return {
        'output_frame': rospy.get_param('~task/output_frame',
                                        'camera_optical_frame'),
        'ar_marker_topic': rospy.get_param('~task/ar_marker_topic',
                                           '/ar_pose_marker'),
        'odom_topic': rospy.get_param('~task/odom_topic', '/odom'),
        'control_rate': rospy.get_param('~task/control_rate', 20.0),
        'kp': rospy.get_param('~task/kp', 1.0),
        'max_angular_rate': rospy.get_param('~task/max_angular_rate', 0.35),
        'search_rate': rospy.get_param('~task/search_rate', 0.15),
        'align_tolerance': rospy.get_param('~task/align_tolerance', 0.035),
        'stable_frames': rospy.get_param('~task/stable_frames', 3),
        'detection_max_age': rospy.get_param('~task/detection_max_age', 0.3),
        'receive_max_age': rospy.get_param('~task/receive_max_age', 0.5),
        'task_timeout': rospy.get_param('~task/task_timeout', 20.0),
        'settle_timeout': rospy.get_param('~task/settle_timeout', 3.0),
        'odom_max_age': rospy.get_param('~task/odom_max_age', 0.5),
        'odom_max_linear': rospy.get_param('~task/odom_max_linear', 0.05),
        'odom_max_angular': rospy.get_param('~task/odom_max_angular', 0.05),
        'stop_publish_count': rospy.get_param('~task/stop_publish_count', 3),
        'stop_publish_interval':
            rospy.get_param('~task/stop_publish_interval', 0.05),
        'shooter': {
            'topic': rospy.get_param('~task/shooter/topic', '/shoot'),
            'pulse_duration':
                rospy.get_param('~task/shooter/pulse_duration', 0.2),
            'delay': rospy.get_param('~task/shooter/delay', 0.1),
            'success': rospy.get_param('~task/shooter/success', True),
            'failure_reason':
                rospy.get_param('~task/shooter/failure_reason',
                                'simulated failure'),
        },
    }


def _task_is_positive_number(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return False
    if math.isnan(v) or math.isinf(v):
        return False
    return v > 0


def _task_is_nonneg_number(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return False
    if math.isnan(v) or math.isinf(v):
        return False
    return v >= 0


def validate_task_config(tc):
    """Validate the AR task parameters (called only when execute_tasks=true).

    Runs before any turning/shooting: an invalid task config must exit with a
    clear error and produce no motion or shot commands.
    """
    if not isinstance(tc, dict):
        return ['task config must be a dict']
    errors = []

    for key in ('output_frame', 'ar_marker_topic', 'odom_topic'):
        v = tc.get(key)
        if not isinstance(v, str) or not v:
            errors.append('task %s must be a non-empty string' % key)

    shooter = tc.get('shooter')
    if not isinstance(shooter, dict):
        errors.append('task shooter must be a dict')
    else:
        if not isinstance(shooter.get('topic'), str) or not shooter['topic']:
            errors.append('task shooter.topic must be a non-empty string')
        if not isinstance(shooter.get('success'), bool):
            errors.append('task shooter.success must be a boolean')
        if not _task_is_positive_number(shooter.get('pulse_duration')):
            errors.append('task shooter.pulse_duration must be a positive '
                          'finite number')
        if not _task_is_nonneg_number(shooter.get('delay')):
            errors.append('task shooter.delay must be a non-negative finite '
                          'number')

    for key in ('control_rate', 'kp', 'max_angular_rate', 'search_rate',
                'align_tolerance', 'detection_max_age', 'receive_max_age',
                'task_timeout', 'settle_timeout', 'odom_max_age'):
        if not _task_is_positive_number(tc.get(key)):
            errors.append('task %s must be a positive finite number' % key)

    for key in ('odom_max_linear', 'odom_max_angular'):
        if not _task_is_nonneg_number(tc.get(key)):
            errors.append('task %s must be a non-negative finite number' % key)

    for key in ('stable_frames', 'stop_publish_count'):
        v = tc.get(key)
        if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
            errors.append('task %s must be a positive integer' % key)

    if not _task_is_positive_number(tc.get('stop_publish_interval')):
        errors.append('task stop_publish_interval must be a positive finite '
                      'number')

    sr = tc.get('search_rate')
    mr = tc.get('max_angular_rate')
    if (_task_is_positive_number(sr) and _task_is_positive_number(mr) and
            sr > mr):
        errors.append('task search_rate must not exceed max_angular_rate')

    return errors


def validate_config(cfg):
    """Return a list of error strings (empty means valid)."""
    errors = []

    frame_id = cfg.get('frame_id')
    if not isinstance(frame_id, str) or not frame_id:
        errors.append('frame_id must be a non-empty string')

    for key in ('ready_timeout', 'server_timeout', 'goal_timeout',
                'cancel_timeout', 'dwell_time'):
        v = cfg.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            errors.append('%s must be a number' % key)
        elif not (v > 0) or math.isnan(v) or math.isinf(v):
            errors.append('%s must be a positive finite number' % key)

    mr = cfg.get('max_retries')
    if isinstance(mr, bool) or not isinstance(mr, int) or mr < 0:
        errors.append('max_retries must be a non-negative integer')

    if cfg.get('failure_policy') not in _VALID_POLICIES:
        errors.append('failure_policy must be one of stop/skip')

    if not isinstance(cfg.get('execute_tasks'), bool):
        errors.append('execute_tasks must be a boolean')

    wps = cfg.get('waypoints')
    if not isinstance(wps, list) or not wps:
        errors.append('waypoints must be a non-empty list')
        return errors

    names = set()
    for i, wp in enumerate(wps):
        if not isinstance(wp, dict):
            errors.append('waypoint %d must be a dict' % i)
            continue
        name = wp.get('name')
        if not isinstance(name, str) or not name:
            errors.append('waypoint %d has no non-empty name' % i)
        elif name in names:
            errors.append('duplicate waypoint name %r' % name)
        else:
            names.add(name)
        kind = wp.get('kind')
        if not isinstance(kind, str) or not kind:
            errors.append('waypoint %d has no non-empty kind '
                          '(task/transit/finish)' % i)
        elif kind not in _VALID_KINDS:
            errors.append('waypoint %d kind must be task/transit/finish' % i)
        if kind == 'finish' and i != len(wps) - 1:
            errors.append('finish waypoint must be the last waypoint')
        for key in ('x', 'y', 'yaw'):
            v = wp.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                errors.append('waypoint %d %s must be a number' % (i, key))
            elif math.isnan(v) or math.isinf(v):
                errors.append('waypoint %d %s must be finite' % (i, key))

    competition = cfg.get('competition', {})
    if not isinstance(competition, dict):
        errors.append('competition must be a dict')
        competition = {}
    if not competition.get('enabled') and any(
            isinstance(wp, dict) and 'target_source' in wp for wp in wps):
        errors.append('target_source waypoints require competition.enabled=true')
    if competition.get('enabled'):
        from aigc_navigation.competition import validate_competition_config
        errors.extend(validate_competition_config(competition, wps))
        if not cfg.get('execute_tasks'):
            errors.append('competition requires execute_tasks=true')

    if cfg.get('execute_tasks') and not competition.get('enabled'):
        # Task mode: every task waypoint must resolve to a valid target id.
        for i, wp in enumerate(wps):
            if not isinstance(wp, dict) or wp.get('kind') != 'task':
                continue
            tid = _resolve_target_id(wp)
            if isinstance(tid, bool) or not isinstance(tid, int) or tid < 0:
                errors.append('task waypoint %d (%r) has no valid target_id'
                              % (i, wp.get('name')))

    if not isinstance(wps[-1], dict) or wps[-1].get('kind') != 'finish':
        errors.append('the last waypoint must be kind: finish')

    return errors


def main():
    rospy.init_node('mission_manager')

    try:
        cfg = load_config()
    except Exception as e:
        rospy.logerr('Failed to read mission parameters: %s' % e)
        sys.exit(1)

    # Resolve the default log directory if the launch file did not provide one.
    if not cfg['log_dir']:
        cfg['log_dir'] = os.path.expanduser('~/.ros/aigc')

    runner = MissionRunner(cfg)
    rospy.on_shutdown(runner.cleanup)

    try:
        errors = validate_config(cfg)
        if cfg.get('execute_tasks'):
            runner.task_cfg = load_task_config()
            errors.extend(validate_task_config(runner.task_cfg))

        if errors:
            rospy.logerr('Invalid configuration: %s' % '; '.join(errors))
            runner.save_record(1, STATUS_FAILED, 'config error')
            sys.exit(1)

        exit_code, final_status, reason = runner.run()
    except KeyboardInterrupt:
        rospy.logwarn('Interrupted; cleaning up.')
        runner.cleanup()
        runner.record_interrupt('interrupted')
        runner.save_record(130, STATUS_STOPPED, 'interrupted')
        sys.exit(130)
    except Exception as e:
        rospy.logerr('Unexpected error: %s' % e)
        runner.cleanup()
        runner.save_record(2, STATUS_FAILED, 'exception: %s' % e)
        sys.exit(2)

    runner.save_record(exit_code, final_status, reason)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
