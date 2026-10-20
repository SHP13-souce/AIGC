# -*- coding: utf-8 -*-
"""Simulated shooting backend (round 4).

Speaks the original shooting interface (``/shoot``, ``std_msgs/String``) with
the C++ commands ``shoot`` / ``stopshoot``, but runs entirely in-process: no
serial port and no real shooting driver are loaded. It only simulates the
state transition so the mission flow can be tested end-to-end.

The returned feedback is a *simulated* result used for state-transition
testing. There is no ballistics or ray-cast hit detection; ``hit`` is always
None and the record must not claim a real hit. ``status`` is one of
``success`` / ``failed`` / ``interrupted`` / ``timeout``: an interrupted or
timed-out shot is never reported as success, even when the configured
``success`` is true.

Python 2.7 / Python 3 compatible.
"""
import time

import rospy
from std_msgs.msg import String

from aigc_navigation.navigation_client import wall_time, is_shutdown


class SimShooter(object):
    """In-process simulated shooter exposing the original /shoot interface."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._pub = rospy.Publisher(cfg['topic'], String, queue_size=1)
        self._pulse_active = False

    # -- pulse / command interface ---------------------------------------
    def _publish(self, text):
        """Publish one command; return the error string, or None on success."""
        try:
            self._pub.publish(String(text))
            return None
        except Exception as e:
            return str(e)

    def stop_pulse(self):
        """Publish ``stopshoot`` if a pulse is still active (idempotent).

        Returns the error string if the stop command could not be published,
        or None on success / when there was no active pulse. The pulse flag is
        cleared either way, so a failed stop never retries forever.
        """
        if not self._pulse_active:
            return None
        err = self._publish('stopshoot')
        self._pulse_active = False
        return err

    def _wait(self, seconds, deadline):
        """Wait ``seconds``, bounded by ``deadline`` and shutdown.

        Returns 'ok' (waited the full duration), 'interrupted' (shutdown), or
        'timeout' (the whole-task deadline passed first). Uses the wall clock
        only, so a paused /clock cannot stall it.
        """
        end = wall_time() + seconds
        while wall_time() < end and wall_time() < deadline:
            if is_shutdown():
                return 'interrupted'
            time.sleep(0.02)
        if is_shutdown():
            return 'interrupted'
        if wall_time() >= deadline:
            return 'timeout'
        return 'ok'

    @staticmethod
    def _feedback(run_id, waypoint, target_id, shot_id, status, reason):
        return {
            'run_id': run_id,
            'waypoint': waypoint,
            'target_id': target_id,
            'shot_id': shot_id,
            'backend': 'simulated',
            'status': status,
            'success': status == 'success',
            'reason': reason,
            'hit': None,
        }

    # -- shoot ------------------------------------------------------------
    def shoot(self, run_id, waypoint, target_id, shot_id, deadline):
        """Trigger one simulated shot and return structured feedback.

        ``deadline`` is the absolute wall-clock deadline of the whole task: the
        pulse and the feedback delay both stop when it passes, and an already
        expired deadline or shutdown at call time means no ``shoot`` is
        published at all. The pulse is always ended via try/finally.
        """
        if is_shutdown():
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'interrupted', 'interrupted')
        if wall_time() >= deadline:
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'timeout', 'timeout')

        start_err = self._publish('shoot')
        if start_err is not None:
            # The start command never went out: nothing was fired, and a shot
            # that could not be started is never reported as success.
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'failed', 'publish shoot failed: %s' %
                                  start_err)

        self._pulse_active = True
        stop_err = None
        try:
            pulse_status = self._wait(self.cfg['pulse_duration'], deadline)
            stop_err = self.stop_pulse()
            if pulse_status != 'ok':
                return self._feedback(run_id, waypoint, target_id, shot_id,
                                      pulse_status, pulse_status)

            delay_status = self._wait(self.cfg['delay'], deadline)
            if delay_status != 'ok':
                return self._feedback(run_id, waypoint, target_id, shot_id,
                                      delay_status, delay_status)
        finally:
            stop_err = self.stop_pulse() or stop_err

        # Re-check after the waits: an interrupt/timeout must not inherit the
        # configured success=true, and a failed stop command is not success.
        if is_shutdown():
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'interrupted', 'interrupted')
        if wall_time() >= deadline:
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'timeout', 'timeout')
        if stop_err is not None:
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'failed', 'publish stopshoot failed: %s' %
                                  stop_err)
        if not self.cfg['success']:
            return self._feedback(run_id, waypoint, target_id, shot_id,
                                  'failed', self.cfg['failure_reason'])
        return self._feedback(run_id, waypoint, target_id, shot_id,
                              'success', '')

    def cleanup(self):
        """End any outstanding pulse (safe to call repeatedly)."""
        self.stop_pulse()
