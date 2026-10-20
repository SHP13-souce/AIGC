# -*- coding: utf-8 -*-
"""Simulated shooting backend (round 4).

Speaks the original shooting interface (``/shoot``, ``std_msgs/String``) with
the C++ commands ``shoot`` / ``stopshoot``, but runs entirely in-process: no
serial port and no real shooting driver are loaded. It only simulates the
state transition so the mission flow can be tested end-to-end.

The returned feedback is a *simulated* result used for state-transition
testing. There is no ballistics or ray-cast hit detection; ``hit`` is always
None and the record must not claim a real hit.

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
        self._pulse_end = 0.0

    # -- pulse / command interface ---------------------------------------
    def _publish(self, text):
        try:
            self._pub.publish(String(text))
        except Exception:
            pass

    def start_pulse(self):
        """Publish ``shoot`` and arm the (interruptible) pulse timer."""
        self._publish('shoot')
        self._pulse_active = True
        self._pulse_end = wall_time() + self.cfg['pulse_duration']

    def stop_pulse(self):
        """Publish ``stopshoot`` if a pulse is still active (idempotent)."""
        if self._pulse_active:
            self._publish('stopshoot')
            self._pulse_active = False

    def _interruptible_wait(self, seconds):
        deadline = wall_time() + seconds
        while wall_time() < deadline:
            if is_shutdown():
                break
            time.sleep(0.02)

    # -- shoot ------------------------------------------------------------
    def shoot(self, run_id, waypoint, target_id, shot_id):
        """Trigger one simulated shot and return structured feedback.

        The pulse (``shoot`` -> ``stopshoot``) and the configured feedback
        delay are both interruptible. The feedback is built fresh on every
        call and never reuses a previous task's result.
        """
        self.start_pulse()
        self._interruptible_wait(self.cfg['pulse_duration'])
        self.stop_pulse()
        self._interruptible_wait(self.cfg['delay'])

        success = bool(self.cfg['success'])
        return {
            'run_id': run_id,
            'waypoint': waypoint,
            'target_id': target_id,
            'shot_id': shot_id,
            'backend': 'simulated',
            'success': success,
            'reason': '' if success else self.cfg['failure_reason'],
            'hit': None,
        }

    def cleanup(self):
        """End any outstanding pulse (safe to call repeatedly)."""
        self.stop_pulse()
