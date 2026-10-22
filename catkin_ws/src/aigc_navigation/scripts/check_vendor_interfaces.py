#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only ROS graph inspection; no service inference/motion/shoot commands."""
import sys
import rospy
import rosservice


def main():
    rospy.init_node('aigc_vendor_interface_check', anonymous=True)
    expected = [('/odom', 'nav_msgs/Odometry'),
                ('/scan', 'sensor_msgs/LaserScan'),
                ('/usb_cam/image_raw', 'sensor_msgs/Image'),
                ('/ar_pose_marker', 'ar_track_alvar_msgs/AlvarMarkers'),
                ('/object_position', 'geometry_msgs/Point'),
                ('/move_base/status', 'actionlib_msgs/GoalStatusArray')]
    published = dict(rospy.get_published_topics('/'))
    failed = False
    for topic, wanted in expected:
        actual = published.get(topic)
        ok = actual == wanted
        print('%s %s: %s (expected %s)' %
              ('OK' if ok else 'MISSING/TYPE ERROR', topic, actual, wanted))
        failed = failed or not ok
    service_type = rosservice.get_service_type('/fruit_detection')
    ok = service_type == 'std_srvs/Trigger'
    print('%s /fruit_detection: %s' % ('OK' if ok else 'MISSING/TYPE ERROR',
                                      service_type))
    failed = failed or not ok
    backend = rospy.get_param('~voice_backend', 'topic')
    if backend == 'service':
        ok = rosservice.get_service_type('/tts_service') == 'TTS_audio/StringService'
        print('%s /tts_service (TTS_audio/StringService)' %
              ('OK' if ok else 'MISSING/TYPE ERROR'))
    elif backend == 'topic':
        code, message, state = rospy.get_master().getSystemState()
        ok = code == 1 and bool(dict(state[1]).get('/voiceWords'))
        print('%s /voiceWords subscriber' % ('OK' if ok else 'MISSING'))
    else:
        print('ERROR: voice_backend must be topic or service')
        ok = False
    failed = failed or not ok
    print('Graph inspection only: recognition content, image freshness, '
          'navigation quality, sound and hardware feedback are NOT tested.')
    return 1 if failed else 0


if __name__ == '__main__':
    try:
        result = main()
    except Exception as exc:
        print('Interface check failed: %s' % exc)
        result = 1
    sys.exit(result)
