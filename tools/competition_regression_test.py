#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vendor-interface adapter tests with ROS substitutes, NOT ROS/hardware tests."""
import copy
import json
import os
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

REAL_MONOTONIC = time.monotonic
import regression_test as base
from aigc_navigation import competition as comp
from aigc_navigation.object_task_executor import ObjectTaskExecutor

PARAMS = {}
RESPONSES = []
RPC_CALLS = []


class ConnectedPublisher(base.Publisher):
    connections = 1

    def __init__(self, *args, **kwargs):
        kwargs.pop('latch', None)
        super().__init__(*args, **kwargs)

    def get_num_connections(self):
        return self.connections


def response(text, success=True):
    return types.SimpleNamespace(message=text, success=success)


def service_request():
    RPC_CALLS.append('read')
    if PARAMS.get('/detect') == 1:
        PARAMS['/detect'] = 255
    return RESPONSES.pop(0)


def interfaces():
    return dict(enabled=True, dry_run=True, profile_confirmed=True,
                vision_service='/fruit_detection', detect_param='/detect',
                recognition_timeout=2.0, service_timeout=0.5, poll_interval=0.1,
                voice_backend='topic', voice_topic='/voiceWords',
                voice_service='/tts_service', voice_timeout=0.3,
                voice_success_values=['TTS处理完成'],
                object_topic='/object_position', object_target_id=1,
                image_width=640, image_height=480, object_center_x=320.0,
                object_kp=0.01, object_tolerance_px=3.0,
                object_receive_max_age=0.3,
                dry_run_shoot_topic='/aigc/competition/dry_run/shoot')


def mission():
    cfg = base.mission_manager.load_config()
    cfg['execute_tasks'] = True
    cfg['competition'] = interfaces()
    cfg['waypoints'] = [dict(name='task_%d' % (index + 1), kind='task',
                             target_source=source, x=0.0, y=0.0, yaw=0.0)
                        for index, source in enumerate(('object', 'math', 'fruit'))]
    cfg['waypoints'].append(dict(name='finish', kind='finish', x=0.0, y=0.0, yaw=0.0))
    return cfg


class CompetitionTests(unittest.TestCase):
    def setUp(self):
        base.reset()
        base.CLOCK.now = base.CLOCK.rostime = 10.0
        PARAMS.clear()
        PARAMS['/detect'] = 255
        RESPONSES[:] = []
        RPC_CALLS[:] = []
        ConnectedPublisher.connections = 1
        self.patches = [
            patch.object(base._rospy, 'Publisher', ConnectedPublisher),
            patch.object(base._rospy, 'get_param', lambda n, d=None: PARAMS.get(n, d)),
            patch.object(base._rospy, 'set_param', lambda n, v: PARAMS.__setitem__(n, v)),
            patch.object(base._rospy, 'wait_for_service', lambda *a, **k: None, create=True),
            patch.object(base._rospy, 'ServiceProxy', lambda *a, **k: service_request, create=True),
        ]
        for p in self.patches:
            p.start()
        pkg = types.ModuleType('std_srvs')
        srv = types.ModuleType('std_srvs.srv')
        srv.Trigger = object
        self.modules_patch = patch.dict('sys.modules', {'std_srvs': pkg, 'std_srvs.srv': srv})
        self.modules_patch.start()

    def tearDown(self):
        self.modules_patch.stop()
        for p in reversed(self.patches):
            p.stop()

    def test_strict_dual_result_protocol(self):
        self.assertEqual(comp.parse_selection(' 2,7 '), {'math': 2, 'fruit': 7})
        self.assertIsNone(comp.parse_selection('0,0'))
        for text in ('31', '1', '0,6', '6,7', '2,9', 'nan,7', '2,7,8', '', '2.0,7'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                comp.parse_selection(text)

    def test_reject_old_cache_without_triggering(self):
        RESPONSES[:] = [response('2,7')]
        result = comp.VisionClient(interfaces()).acquire()
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(PARAMS['/detect'], 255)
        self.assertEqual(len(RPC_CALLS), 1)

    def test_trigger_poll_new_pair_and_reset(self):
        RESPONSES[:] = [response('0,0'), response('0,0'), response('3,8')]
        result = comp.VisionClient(interfaces()).acquire()
        self.assertEqual(result['selection'], {'math': 3, 'fruit': 8})
        self.assertEqual(PARAMS['/detect'], 255)

    def test_service_failure_and_single_result_never_select(self):
        for item in (response('2,7', False), response('31')):
            RESPONSES[:] = [item]
            self.assertEqual(comp.VisionClient(interfaces()).acquire()['status'], 'failed')
            self.assertEqual(PARAMS['/detect'], 255)

    def test_pending_trigger_refused(self):
        RESPONSES[:] = [response('0,0')]
        PARAMS['/detect'] = 1
        self.assertEqual(comp.VisionClient(interfaces()).acquire()['status'], 'failed')

    def test_recognition_timeout_and_interruption_cancel_trigger(self):
        for interrupted in (False, True):
            base.CLOCK.shutdown = False
            reads = [0]
            def read_empty(deadline):
                reads[0] += 1
                if reads[0] > 2:
                    if interrupted:
                        base.CLOCK.shutdown = True
                    else:
                        base.CLOCK.now += 3.0
                return comp.outcome('success', selection=None)
            client = comp.VisionClient(interfaces())
            client._read = read_empty
            result = client.acquire()
            self.assertEqual(result['status'], 'interrupted' if interrupted else 'timeout')
            self.assertEqual(PARAMS['/detect'], 255)

    def test_blocked_service_wall_deadline_and_late_result(self):
        release = threading.Event()
        started = threading.Event()
        def blocked():
            started.set()
            release.wait(1.0)
            return 'late'
        with patch.object(comp, 'wall_time', REAL_MONOTONIC):
            begin = REAL_MONOTONIC()
            result = comp.bounded_call(blocked, begin + 0.04)
            elapsed = REAL_MONOTONIC() - begin
        release.set()
        self.assertTrue(started.is_set())
        self.assertEqual(result['status'], 'timeout')
        self.assertLess(elapsed, 0.5)
        self.assertNotIn('value', result)

    def test_shutdown_prevents_rpc(self):
        base.CLOCK.shutdown = True
        called = []
        result = comp.bounded_call(lambda: called.append(True), 20.0)
        self.assertEqual(result['status'], 'interrupted')
        self.assertEqual(called, [])

    def test_profile_and_physical_shoot_topic_validation(self):
        cfg = mission()
        self.assertEqual(base.mission_manager.validate_config(cfg), [])
        for key, value in (('dry_run', False), ('profile_confirmed', False),
                           ('dry_run_shoot_topic', '/shoot'),
                           ('dry_run_shoot_topic', '//shoot/'),
                           ('object_target_id', 255), ('object_center_x', float('nan')),
                           ('image_width', 0), ('recognition_timeout', float('inf'))):
            modified = copy.deepcopy(cfg)
            modified['competition'][key] = value
            self.assertTrue(base.mission_manager.validate_config(modified), key)

    def test_target_order_and_transit_waypoints(self):
        cfg = mission()
        cfg['competition']['enabled'] = False
        self.assertTrue(base.mission_manager.validate_config(cfg))
        cfg['competition']['enabled'] = True
        cfg['waypoints'].insert(1, dict(name='transit', kind='transit', x=0, y=0, yaw=0))
        self.assertEqual(base.mission_manager.validate_config(cfg), [])
        cfg['waypoints'][2]['target_source'] = 'fruit'
        self.assertTrue(base.mission_manager.validate_config(cfg))

    def test_voice_requires_connected_subscriber_and_records_publication(self):
        voice = comp.VoiceClient(interfaces())
        ConnectedPublisher.connections = 0
        self.assertEqual(voice.say('test')['status'], 'timeout')
        self.assertEqual(base.published_data('/voiceWords'), [])
        ConnectedPublisher.connections = 1
        result = voice.say('test')
        self.assertEqual(result['delivery'], 'published')
        self.assertIsNone(result['audible'])
        self.assertEqual(base.published_data('/voiceWords'), ['test'])

    def point_executor(self):
        executor = ObjectTaskExecutor(base.task_cfg(), interfaces())
        executor._point_start_wall = base.CLOCK.now - 1
        return executor

    def test_point_protocol_ids_bounds_loss_and_receive_age(self):
        ex = self.point_executor()
        for x, y, z, valid in ((320, 240, 1, True), (320, 240, 2, False),
                               (320, 240, 255, False), (640, 240, 1, False),
                               (320, -1, 1, False), (float('nan'), 240, 1, False)):
            msg = base.Point()
            msg.x, msg.y, msg.z = x, y, z
            ex._on_point(msg)
            self.assertEqual(ex._latest_valid_detection(1) is not None, valid)
        msg.x, msg.y, msg.z = 320, 240, 1
        ex._on_point(msg)
        base.CLOCK.now += 0.4
        self.assertIsNone(ex._latest_valid_detection(1))

    def test_one_point_sample_cannot_accumulate_stable_frames(self):
        ex = self.point_executor()
        msg = base.Point()
        msg.x, msg.y, msg.z = 320, 240, 1
        ex._on_point(msg)
        base.SLEEP_HOOKS.append(lambda: ex._on_odom(base.make_odom(base.CLOCK.rostime)))
        result = ex._search_align(1, base.CLOCK.now + 0.5)
        self.assertEqual(result, 'timeout')
        self.assertEqual(base.published_data('/shoot'), [])

    def test_template_turn_uses_pixel_error_with_clamped_speed(self):
        ex = self.point_executor()
        msg = base.Point()
        msg.x, msg.y, msg.z = 420, 240, 1
        ex._on_point(msg)
        ex._search_align(1, base.CLOCK.now + 0.04)
        turns = [m.angular.z for t, m in base.PUBLISHED if t == '/cmd_vel' and m.angular.z]
        self.assertEqual(turns[0], -0.35)

    def test_template_final_gate_rechecks_pixel_alignment(self):
        ex = self.point_executor()
        msg = base.Point()
        msg.x, msg.y, msg.z = 350, 240, 1
        ex._on_point(msg)
        ex._on_odom(base.make_odom(base.CLOCK.rostime))
        result = ex._final_gate_and_shoot('run', 'task_1', 1, True, 20.0)
        self.assertEqual(result['status'], 'failed')
        self.assertIn('moved', result['reason'])
        self.assertEqual(base.published_data('/shoot'), [])

    def test_voice_service_uses_vendor_type_and_checks_acknowledgement(self):
        cfg = interfaces()
        cfg['voice_backend'] = 'service'
        message_module = types.ModuleType('roslib.message')
        message_module.get_service_class = lambda name: object
        with patch.dict('sys.modules', {'roslib': types.ModuleType('roslib'),
                                        'roslib.message': message_module}):
            for reply, wanted in (('TTS处理完成', 'success'), ('错误: unavailable', 'failed')):
                with patch.object(base._rospy, 'ServiceProxy',
                                  lambda *a, **k: lambda text: types.SimpleNamespace(result=reply)):
                    result = comp.VoiceClient(cfg).say('test')
                    self.assertEqual(result['status'], wanted)

    def test_final_announcement_failure_prevents_completed_status(self):
        cfg = mission()
        base.GOAL_RESULTS[:] = [base.GoalStatus.SUCCEEDED] * 4
        integration = types.SimpleNamespace(
            prepare=lambda waypoints: [wp.update(target_id=1) for wp in waypoints] or None,
            execute_task=lambda *args: dict(status='success', aligned=True, shot=None, reason=''),
            announce_finish=lambda: comp.outcome('timeout', 'voice timeout'),
            cleanup=lambda: None)
        def prepare(waypoints):
            for wp in waypoints:
                wp['target_id'] = 1
            return comp.outcome('success')
        integration.prepare = prepare
        runner = base.mission_manager.MissionRunner(cfg)
        runner.task_cfg = base.task_cfg()
        with patch.object(comp, 'CompetitionIntegration', lambda *args: integration):
            result = runner.run()
        self.assertEqual(result, (2, 'FAILED', 'voice timeout'))
        self.assertEqual(runner.records[-1]['result'], 'failed')
        self.assertEqual(runner.records[-1]['navigation_result'], 'success')

    def test_shutdown_after_finish_announcement_is_stopped(self):
        cfg = mission()
        base.GOAL_RESULTS[:] = [base.GoalStatus.SUCCEEDED] * 4
        def prepare(waypoints):
            for wp in waypoints:
                wp['target_id'] = 1
            return comp.outcome('success')
        def announce():
            base.CLOCK.shutdown = True
            return comp.outcome('success')
        integration = types.SimpleNamespace(
            prepare=prepare,
            execute_task=lambda *args: dict(status='success', aligned=True, shot=None, reason=''),
            announce_finish=announce, cleanup=lambda: None)
        runner = base.mission_manager.MissionRunner(cfg)
        runner.task_cfg = base.task_cfg()
        with patch.object(comp, 'CompetitionIntegration', lambda *args: integration):
            result = runner.run()
        self.assertEqual(result[0:2], (130, 'STOPPED'))

    def test_malformed_competition_config_still_saves_failure_record(self):
        cfg = mission()
        cfg['competition'] = []
        self.assertTrue(base.mission_manager.validate_config(cfg))
        runner = base.mission_manager.MissionRunner(cfg)
        with tempfile.TemporaryDirectory() as directory:
            cfg['log_dir'] = directory
            runner.save_record(1, 'FAILED', 'config error')
            self.assertTrue(os.path.isfile(os.path.join(directory, runner.run_id + '.json')))

    def test_whole_mission_dynamic_targets_order_record_and_no_physical_shots(self):
        cfg = mission()
        RESPONSES[:] = [response('0,0'), response('2,7')]
        base.GOAL_RESULTS[:] = [base.GoalStatus.SUCCEEDED] * 4
        runner = base.mission_manager.MissionRunner(cfg)
        runner.task_cfg = base.task_cfg()
        def feed_sensors():
            if runner.competition is None:
                return
            for key, ex in runner.competition.executors.items():
                ex._on_odom(base.make_odom(base.CLOCK.rostime))
                if key == 'object':
                    point = base.Point()
                    point.x, point.y, point.z = 320, 240, 1
                    ex._on_point(point)
                else:
                    ex._on_markers(base.make_markers([
                        base.make_marker(2, base.CLOCK.rostime),
                        base.make_marker(7, base.CLOCK.rostime)]))
        base.SLEEP_HOOKS.append(feed_sensors)
        result = runner.run()
        self.assertEqual(result, (0, 'COMPLETED', ''))
        self.assertEqual([w.get('target_id') for w in runner.records], [1, 2, 7, None])
        self.assertEqual([w['name'] for w in runner.records], ['task_1', 'task_2', 'task_3', 'finish'])
        self.assertEqual(base.published_data('/shoot'), [])
        self.assertEqual(base.published_data('/aigc/competition/dry_run/shoot'),
                         ['shoot', 'stopshoot'] * 3)
        self.assertEqual(len(base.published_data('/voiceWords')), 2)
        self.assertEqual(runner.records[0]['task']['detection_freshness'],
                         'receive_time_only_no_image_header')
        with tempfile.TemporaryDirectory() as directory:
            cfg['log_dir'] = directory
            runner.save_record(*result)
            with open(os.path.join(directory, runner.run_id + '.json')) as fh:
                record = json.load(fh)
            self.assertEqual(record['mission_mode'], 'competition_dry_run')
            self.assertEqual(record['integration']['selection'], {'math': 2, 'fruit': 7})

    def test_recognition_failure_prevents_any_goal(self):
        RESPONSES[:] = [response('31')]
        runner = base.mission_manager.MissionRunner(mission())
        runner.task_cfg = base.task_cfg()
        self.assertEqual(runner.run()[0], 2)
        self.assertEqual(runner.client._client.sent, [])
        self.assertEqual(base.published_data('/shoot'), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
