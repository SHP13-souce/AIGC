# -*- coding: utf-8 -*-
"""Vendor /object_position Point: x/y=pixel center, z=template ID.

The original message has NO header. Freshness is receive-time only, unlike
the AR image-header checks; it cannot certify image capture time. Keep this
limitation explicit in the task record. z=255 means no detection.
"""
import copy
import math
import threading

import rospy
from geometry_msgs.msg import Point
from aigc_navigation.navigation_client import wall_time
from aigc_navigation.task_executor import TaskExecutor


class _Detection(object):
    pass


class ObjectTaskExecutor(TaskExecutor):
    def __init__(self, task_cfg, competition_cfg):
        cfg = copy.deepcopy(task_cfg)
        cfg['kp'] = competition_cfg['object_kp']
        cfg['align_tolerance'] = competition_cfg['object_tolerance_px']
        self.object_cfg = competition_cfg
        self._point_lock = threading.Lock()
        self._point_sample = None
        self._point_start_wall = float('inf')
        TaskExecutor.__init__(self, cfg)
        self._subs.append(rospy.Subscriber(competition_cfg['object_topic'],
                                           Point, self._on_point))

    def _on_point(self, msg):
        with self._point_lock:
            # One immutable snapshot prevents mixed coordinate/stamp reads.
            self._point_sample = (msg.x, msg.y, msg.z, wall_time(),
                                   rospy.get_rostime())

    def _latest_valid_detection(self, target_id):
        with self._point_lock:
            sample = self._point_sample
        if sample is None:
            return None
        x, y, template_id, received, stamp = sample
        cfg = self.object_cfg
        if not all(isinstance(v, (int, float)) and not math.isnan(v) and
                   not math.isinf(v) for v in (x, y, template_id)):
            return None
        age = wall_time() - received
        if (received <= self._point_start_wall or age < 0 or
                age > cfg['object_receive_max_age'] or
                template_id == 255 or template_id != target_id or
                not 0 <= x < cfg['image_width'] or
                not 0 <= y < cfg['image_height'] or
                not self._valid_stamp(stamp)):
            return None
        detection = _Detection()
        detection.pixel_x = x
        detection.header = _Detection()
        detection.header.stamp = stamp
        return detection

    def _alignment_error(self, detection):
        return detection.pixel_x - self.object_cfg['object_center_x']

    def execute(self, run_id, waypoint, target_id, allow_shot=True):
        with self._point_lock:
            self._point_sample = None
            self._point_start_wall = wall_time()
        result = TaskExecutor.execute(self, run_id, waypoint, target_id,
                                      allow_shot)
        result['detection_freshness'] = 'receive_time_only_no_image_header'
        return result
