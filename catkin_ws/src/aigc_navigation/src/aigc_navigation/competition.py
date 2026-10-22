# -*- coding: utf-8 -*-
"""Adapters for the supplied ROS1 competition interfaces.

No vendor source, credentials or cloud client is copied here. The vendor
identify/find-object/voice nodes run separately. Shooting remains a named
dry-run backend until a real completion-feedback contract is available.
"""
import copy
import math
import re
import sys
import threading
import time

import rospy
from std_msgs.msg import String
from aigc_navigation.navigation_client import wall_time, is_shutdown

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def outcome(status, reason='', **extra):
    result = {'status': status, 'reason': reason}
    result.update(extra)
    return result


def _ros_text(text):
    # Melodic's Python 2 generated serializers expect UTF-8 bytes; Python 3
    # generated serializers expect unicode text.
    if sys.version_info[0] < 3 and isinstance(text, type(u'')):
        return text.encode('utf-8')
    return text


def _unicode_text(text):
    return text.decode('utf-8') if isinstance(text, bytes) else text


def parse_selection(message):
    """Strict vendor identify.py response: math 1..5, fruit 6..8.

    0,0 is the vendor's empty/failure cache. Single-number responses from
    identify_service.py are a different protocol and must never select a target.
    """
    if not isinstance(message, string_types):
        raise ValueError('recognition response must be text')
    match = re.match(r'^\s*([0-9]+)\s*,\s*([0-9]+)\s*$', message)
    if match is None:
        raise ValueError('expected math,fruit; use the dual-result identify.py')
    math_id, fruit_id = [int(value) for value in match.groups()]
    if (math_id, fruit_id) == (0, 0):
        return None
    if math_id not in range(1, 6) or fruit_id not in range(6, 9):
        raise ValueError('recognition IDs outside math=1..5 / fruit=6..8')
    return {'math': math_id, 'fruit': fruit_id}


def bounded_call(function, deadline):
    """One RPC at a time, bounded by wall time even with a paused ROS clock.

    A timed-out RPC may still execute in the server; its result is discarded.
    The caller stops the mission and never retries that uncertain RPC.
    """
    if is_shutdown():
        return outcome('interrupted', 'shutdown before service call')
    if wall_time() >= deadline:
        return outcome('timeout', 'service deadline expired')
    done = threading.Event()
    box = {}

    def invoke():
        try:
            box['value'] = function()
        except Exception as exc:
            box['error'] = str(exc)
        finally:
            done.set()

    worker = threading.Thread(target=invoke)
    worker.daemon = True
    worker.start()
    while not done.is_set():
        if is_shutdown():
            return outcome('interrupted', 'shutdown during service call')
        if wall_time() >= deadline:
            return outcome('timeout', 'service call timeout; mission stopped')
        done.wait(0.01)
    if is_shutdown():
        return outcome('interrupted', 'shutdown after service call')
    if wall_time() >= deadline:
        return outcome('timeout', 'late service response discarded')
    if 'error' in box:
        return outcome('failed', 'service call failed: ' + box['error'])
    return outcome('success', value=box['value'])


class VisionClient(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.proxy = None

    def _read(self, deadline):
        # Import lazily: navigation-only and the old AR simulation do not need
        # std_srvs or any vendor-specific service package at import time.
        from std_srvs.srv import Trigger

        def request():
            rospy.wait_for_service(self.cfg['vision_service'],
                                   timeout=self.cfg['service_timeout'])
            if self.proxy is None:
                self.proxy = rospy.ServiceProxy(self.cfg['vision_service'],
                                               Trigger)
            return self.proxy()

        result = bounded_call(request, min(deadline, wall_time() +
                                          self.cfg['service_timeout']))
        if result['status'] != 'success':
            return result
        response = result['value']
        if response.success is not True:
            return outcome('failed', 'recognition service returned success=false')
        try:
            return outcome('success', selection=parse_selection(response.message))
        except ValueError as exc:
            return outcome('failed', str(exc))

    def acquire(self):
        deadline = wall_time() + self.cfg['recognition_timeout']
        baseline = self._read(deadline)
        if baseline['status'] != 'success':
            return baseline
        # The vendor response has no timestamp / request ID and /detect is
        # reset BEFORE inference finishes. A pre-existing valid pair cannot be
        # certified as new. Require a fresh identify process with cache=0,0.
        if baseline['selection'] is not None:
            return outcome('failed', 'non-empty recognition cache; restart the '
                           'vendor identify node before a new mission')
        if is_shutdown():
            return outcome('interrupted', 'shutdown before recognition trigger')
        if wall_time() >= deadline:
            return outcome('timeout', 'recognition deadline expired')
        if rospy.get_param(self.cfg['detect_param'], 255) != 255:
            return outcome('failed', 'recognition already pending; restart '
                           'identify and ensure this is the only controller')
        rospy.set_param(self.cfg['detect_param'], 1)
        try:
            while wall_time() < deadline:
                if is_shutdown():
                    return outcome('interrupted', 'recognition interrupted')
                current = self._read(deadline)
                if current['status'] != 'success':
                    return current
                if current['selection'] is not None:
                    if rospy.get_param(self.cfg['detect_param'], 1) != 255:
                        return outcome('failed', 'result received before '
                                       'the recognition trigger was consumed')
                    return current
                time.sleep(self.cfg['poll_interval'])
            return outcome('timeout', 'recognition timeout: no valid new pair')
        finally:
            # Cancel an unconsumed image trigger; never create a second request.
            rospy.set_param(self.cfg['detect_param'], 255)


class VoiceClient(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.events = []
        self.publisher = None
        if cfg['voice_backend'] == 'topic':
            self.publisher = rospy.Publisher(cfg['voice_topic'], String,
                                             queue_size=1, latch=False)

    def say(self, text):
        deadline = wall_time() + self.cfg['voice_timeout']
        if self.cfg['voice_backend'] == 'topic':
            while self.publisher.get_num_connections() == 0:
                if is_shutdown():
                    return outcome('interrupted', 'voice interrupted')
                if wall_time() >= deadline:
                    return outcome('timeout', 'no subscriber on voice topic')
                time.sleep(0.02)
            if is_shutdown():
                return outcome('interrupted', 'voice interrupted')
            if wall_time() >= deadline:
                return outcome('timeout', 'voice deadline expired')
            try:
                self.publisher.publish(String(_ros_text(text)))
            except Exception as exc:
                return outcome('failed', 'voice publication failed: %s' % exc)
            result = outcome('success', delivery='published', audible=None)
        else:
            def request():
                from roslib.message import get_service_class
                service_class = get_service_class('TTS_audio/StringService')
                if service_class is None:
                    raise ValueError('TTS_audio/StringService is unavailable; '
                                     'source the vendor workspace first')
                rospy.wait_for_service(self.cfg['voice_service'],
                                       timeout=self.cfg['voice_timeout'])
                return rospy.ServiceProxy(self.cfg['voice_service'],
                                          service_class)(_ros_text(text))
            result = bounded_call(request, deadline)
            if result['status'] == 'success':
                if _unicode_text(result['value'].result) not in [
                        _unicode_text(v) for v in self.cfg['voice_success_values']]:
                    result = outcome('failed', 'TTS service did not confirm '
                                     'successful processing')
                else:
                    result = outcome('success', delivery='service_completed',
                                     audible=None)
        self.events.append({'text': text, 'backend': self.cfg['voice_backend'],
                            'result': result})
        return result


def validate_competition_config(cfg, waypoints):
    errors = []
    if cfg.get('enabled') is not True:
        errors.append('competition.enabled must be boolean true')
    if cfg.get('dry_run') is not True:
        errors.append('only competition dry_run=true is implemented; '
                      'real shooting feedback has not been specified')
    if cfg.get('profile_confirmed') is not True:
        errors.append('confirm the map/waypoints/template IDs before setting '
                      'competition.profile_confirmed=true')
    for key in ('vision_service', 'detect_param', 'voice_topic', 'voice_service',
                'object_topic', 'dry_run_shoot_topic'):
        value = cfg.get(key)
        if not isinstance(value, string_types) or not value.startswith('/'):
            errors.append('competition.%s must be an absolute ROS name' % key)
    shoot_topic = cfg.get('dry_run_shoot_topic')
    if (isinstance(shoot_topic, string_types) and
            '/' + '/'.join(part for part in shoot_topic.split('/') if part) == '/shoot'):
        errors.append('dry-run shooting must not use the physical /shoot topic')
    for key in ('recognition_timeout', 'service_timeout', 'poll_interval',
                'voice_timeout', 'object_receive_max_age', 'object_kp',
                'object_tolerance_px', 'image_width', 'image_height'):
        value = cfg.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                math.isnan(value) or math.isinf(value) or value <= 0):
            errors.append('competition.%s must be positive and finite' % key)
    center = cfg.get('object_center_x')
    width = cfg.get('image_width')
    if (isinstance(center, bool) or not isinstance(center, (int, float)) or
            math.isnan(center) or math.isinf(center) or
            not isinstance(width, (int, float)) or not 0 <= center < width):
        errors.append('object_center_x must be inside the configured image')
    target = cfg.get('object_target_id')
    if isinstance(target, bool) or not isinstance(target, int) or not 0 <= target < 255:
        errors.append('object_target_id must be a template ID in 0..254')
    if cfg.get('voice_backend') not in ('topic', 'service'):
        errors.append('voice_backend must be topic or service')
    values = cfg.get('voice_success_values')
    if not isinstance(values, list) or not values or not all(
            isinstance(v, string_types) and v for v in values):
        errors.append('voice_success_values must contain non-empty strings')
    tasks = [wp for wp in waypoints if isinstance(wp, dict) and
             wp.get('kind') == 'task']
    if [wp.get('target_source') for wp in tasks] != ['object', 'math', 'fruit']:
        errors.append('competition task order must be object, math, fruit')
    return errors


class CompetitionIntegration(object):
    def __init__(self, cfg, task_cfg):
        self.cfg = cfg
        self.task_cfg = copy.deepcopy(task_cfg)
        self.task_cfg['shooter']['topic'] = cfg['dry_run_shoot_topic']
        self.vision = VisionClient(cfg)
        self.voice = VoiceClient(cfg)
        self.executors = {}
        self.record = {'backend': 'vendor_interfaces_dry_run', 'hit': None,
                       'selection': None, 'voice': self.voice.events}

    def prepare(self, waypoints):
        result = self.vision.acquire()
        self.record['recognition'] = result
        if result['status'] != 'success':
            return result
        selection = result['selection']
        self.record['selection'] = selection
        for wp in waypoints:
            if wp['kind'] != 'task':
                continue
            source = wp['target_source']
            wp['target_id'] = (self.cfg['object_target_id'] if source == 'object'
                               else selection[source])
        return self.voice.say(u'识别完成，数学答案为%d，水果编码为%d。' %
                              (selection['math'], selection['fruit']))

    def execute_task(self, run_id, wp, allow_shot):
        source = wp['target_source']
        key = 'object' if source == 'object' else 'ar'
        if key not in self.executors:
            if key == 'object':
                from aigc_navigation.object_task_executor import ObjectTaskExecutor
                self.executors[key] = ObjectTaskExecutor(self.task_cfg, self.cfg)
            else:
                from aigc_navigation.task_executor import TaskExecutor
                self.executors[key] = TaskExecutor(self.task_cfg)
        result = self.executors[key].execute(run_id, wp['name'], wp['target_id'],
                                              allow_shot)
        result['target_source'] = source
        result['dry_run'] = True
        return result

    def announce_finish(self):
        return self.voice.say(u'已到达终点，任务结束。')

    def cleanup(self):
        for executor in self.executors.values():
            try:
                executor.cleanup()
            except Exception:
                pass
