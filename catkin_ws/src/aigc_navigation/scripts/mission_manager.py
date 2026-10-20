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
import threading
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

# Non-ordinary outcomes surfaced from navigate_to(); each stops the whole run.
OUT_LOST = 'lost'
OUT_CANCEL_UNCONFIRMED = 'cancel_unconfirmed'
OUT_SHUTDOWN = 'shutdown'

_VALID_KINDS = ('task', 'transit', 'finish')
_VALID_POLICIES = ('stop', 'skip')


class MissionRunner(object):
    """Holds per-run state so every restart starts fresh."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.client = NavigationClient('move_base')
        self.tf_listener = tf.TransformListener()
        self.records = []
        self.skipped_any = False
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

    # -- readiness --------------------------------------------------------
    def wait_ready(self):
        """Wait (wall-clock bounded) for the navigation stack to be ready.

        Returns True when all three hold: the action server is connected, the
        map -> base_footprint TF is available, and simulation time has started.
        Never hangs even if /clock stays at zero.
        """
        deadline = wall_time() + self.cfg['ready_timeout']
        frame = self.cfg['frame_id']

        connected = {'ok': False}

        def _connect():
            try:
                connected['ok'] = self.client.wait_for_server_blocking()
            except Exception:
                connected['ok'] = False

        thread = threading.Thread(target=_connect)
        thread.daemon = True
        thread.start()

        server_ok = False
        tf_ok = False
        clock_ok = False
        while wall_time() < deadline:
            if is_shutdown():
                break
            if connected['ok']:
                server_ok = True
            if not tf_ok:
                try:
                    self.tf_listener.waitForTransform(
                        frame, 'base_footprint', rospy.Time(0),
                        rospy.Duration(0.2))
                    tf_ok = True
                except Exception:
                    pass
            if rospy.get_rostime() > rospy.Time(0):
                clock_ok = True
            if server_ok and tf_ok and clock_ok:
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

        attempts = 0
        while True:
            if is_shutdown():
                return {'status': OUT_SHUTDOWN, 'attempts': attempts,
                        'reason': 'shutdown'}

            self.client.send_goal(wp['x'], wp['y'], wp['yaw'], frame_id)
            attempts += 1

            state = self.client.wait_for_result(goal_timeout)

            if is_shutdown():
                return {'status': OUT_SHUTDOWN, 'attempts': attempts,
                        'reason': 'shutdown'}

            if state is None:
                # Execution timed out: cancel, then confirm the old goal has
                # finished before retrying or continuing.
                self.client.cancel_goal()
                if self.client.wait_for_cancel(cancel_timeout):
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

    # -- record -----------------------------------------------------------
    def _record(self, wp, result, attempts, reason):
        self.records.append({
            'name': wp['name'],
            'kind': wp['kind'],
            'x': wp['x'],
            'y': wp['y'],
            'yaw': wp['yaw'],
            'attempts': attempts,
            'result': result,
            'reason': reason,
        })

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

        for wp in self.cfg['waypoints']:
            if is_shutdown():
                self.cleanup()
                return (130, STATUS_STOPPED, 'shutdown')

            rospy.loginfo('Navigating to %s (%s): x=%.3f y=%.3f yaw=%.3f' %
                          (wp['name'], wp['kind'], wp['x'], wp['y'], wp['yaw']))
            res = self.navigate_to(wp)

            if res['status'] == OUT_SHUTDOWN:
                self.cleanup()
                return (130, STATUS_STOPPED, 'shutdown')

            if res['status'] == OUT_LOST:
                self._record(wp, WP_FAILED, res['attempts'], 'lost')
                rospy.logerr('Waypoint %s: LOST (connection unclear); '
                             'stopping the run.' % wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'lost connection')

            if res['status'] == OUT_CANCEL_UNCONFIRMED:
                self._record(wp, WP_FAILED, res['attempts'],
                             'cancel not confirmed')
                rospy.logerr('Waypoint %s: cancel not confirmed; stopping '
                             'the run.' % wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'cancel unconfirmed')

            if res['status'] == WP_SUCCESS:
                self._record(wp, WP_SUCCESS, res['attempts'], '')
                rospy.loginfo('Waypoint %s reached (attempts=%d).' %
                              (wp['name'], res['attempts']))
                if wp['kind'] == 'task':
                    self.on_task_reached(wp)
                continue

            # WP_FAILED: ordinary failure after retries exhausted.
            kind = wp['kind']
            if kind == 'transit':
                self._record(wp, WP_FAILED, res['attempts'], res['reason'])
                rospy.logerr('Transit waypoint %s failed; stopping the run.' %
                             wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'transit failed')
            if kind == 'finish':
                self._record(wp, WP_FAILED, res['attempts'], res['reason'])
                rospy.logerr('Finish waypoint %s failed; run NOT completed.' %
                             wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'finish failed')
            if self.cfg['failure_policy'] == 'stop':
                self._record(wp, WP_FAILED, res['attempts'], res['reason'])
                rospy.logerr('Task waypoint %s failed; stopping '
                             '(failure_policy=stop).' % wp['name'])
                self.cleanup()
                return (2, STATUS_FAILED, 'task failed (stop)')
            # task + skip policy
            self._record(wp, WP_SKIPPED, res['attempts'],
                         'skipped after failure')
            self.skipped_any = True
            rospy.logwarn('Task waypoint %s failed; skipping '
                          '(failure_policy=skip).' % wp['name'])

        if self.skipped_any:
            self.cleanup()
            return (3, STATUS_PARTIAL, 'skipped task waypoints')
        self.cleanup()
        return (0, STATUS_COMPLETED, '')

    # -- save the run record ---------------------------------------------
    def save_record(self, exit_code, final_status, reason):
        record = {
            'run_id': self.run_id,
            'start_time': self.start_time_str,
            'end_time': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'status': final_status,
            'exit_code': exit_code,
            'reason': reason,
            'duration_sec': round(wall_time() - self.start_wall, 3),
            'waypoints': self.records,
        }
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
        'waypoints': rospy.get_param('~waypoints', []),
        'log_dir': rospy.get_param('~log_dir', ''),
    }


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
        kind = wp.get('kind', 'task')
        if kind not in _VALID_KINDS:
            errors.append('waypoint %d kind must be task/transit/finish' % i)
        if kind == 'finish' and i != len(wps) - 1:
            errors.append('finish waypoint must be the last waypoint')
        for key in ('x', 'y', 'yaw'):
            v = wp.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                errors.append('waypoint %d %s must be a number' % (i, key))
            elif math.isnan(v) or math.isinf(v):
                errors.append('waypoint %d %s must be finite' % (i, key))

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
        if errors:
            rospy.logerr('Invalid mission configuration: %s' %
                         '; '.join(errors))
            runner.save_record(1, STATUS_FAILED, 'config error')
            sys.exit(1)

        exit_code, final_status, reason = runner.run()
    except KeyboardInterrupt:
        rospy.logwarn('Interrupted; cleaning up.')
        runner.cleanup()
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
