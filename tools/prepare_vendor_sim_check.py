#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Prepare ONLY AIGC test.world for a real-perception, dry-shoot mission check.

No vendor source or API credential is copied or modified. ROS imports are lazy
so mesh/config generation can be checked without ROS. Use the VLM Python venv.
"""
import argparse
import math
import os
from pathlib import Path
import runpy
import shlex
import shutil
import sys
import xml.etree.ElementTree as ET

MODEL_NAMES = (
    'aigc_vendor_template_mission_check',
    'aigc_vendor_fruit_mission_check',
    'aigc_vendor_vlm_check',
)
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
# ID 6: sampled at the cell centers of the upstream ALVAR sheet, row 3,
# column 1. Same 9x9/9 cm convention as existing marker_0/1/2 in AIGC.
PATTERN_6 = [
    '000000000', '000000000', '001101100', '001101100', '001010100',
    '000011100', '000011100', '000000000', '000000000',
]


def make_mesh(base, target, texture, width, height):
    ET.register_namespace('', NS['c'])
    tree = ET.parse(str(base))
    image = tree.find('.//c:image/c:init_from', NS)
    if image is None:
        raise ValueError('base DAE has no texture image')
    image.text = texture.absolute().as_uri()
    found = False
    for array in tree.findall('.//c:float_array', NS):
        if array.get('id', '').endswith('_pos_arr'):
            x, y = width / 2.0, height / 2.0
            array.text = '{} {} 0 {} {} 0 {} {} 0 {} {} 0'.format(
                -x, -y, x, -y, -x, y, x, y)
            found = True
    if not found:
        raise ValueError('base DAE has no marker position array')
    tree.write(str(target), encoding='utf-8', xml_declaration=True)


def board_sdf(name, mesh, face_yaw):
    # Local XY plane: roll pi/2 gives normal -Y; yaw pi faces +Y.
    root = ET.Element('sdf', version='1.6')
    model = ET.SubElement(root, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='link')
    visual = ET.SubElement(link, 'visual', name='board')
    ET.SubElement(visual, 'pose').text = '0 0 0 {} 0 {}'.format(
        math.pi / 2, face_yaw)
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(ET.SubElement(geometry, 'mesh'), 'uri').text = mesh.absolute().as_uri()
    # Visual-only test overlays retain the existing map/collision geometry.
    return ET.tostring(root, encoding='unicode')


def prepare_files(repo, vendor, output):
    import yaml
    nav = repo / 'catkin_ws/src/aigc_navigation'
    sim = repo / 'catkin_ws/src/aigc_sim'
    base = sim / 'meshes/marker_0.dae'
    template = vendor / 'src/abot_find/shoot_object/113.png'
    sample = vendor / 'src/abot_vlm/temp2/vl_now.jpg'
    for file in (base, template, sample):
        if not file.is_file():
            raise ValueError('missing input: {}'.format(file))
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(str(template), str(output / '113.png'))
    shutil.copyfile(str(sample), str(output / 'vlm_sample.jpg'))
    generator = runpy.run_path(str(repo / 'tools/generate_test_markers.py'))
    rows = generator['marker_rows'](PATTERN_6)
    generator['write_grayscale_png'](str(output / 'marker_6.png'), 180, 180, rows)
    make_mesh(base, output / 'template_113.dae', output / '113.png',
              0.18, 0.18 * 106 / 92)
    make_mesh(base, output / 'marker_6.dae', output / 'marker_6.png', 0.09, 0.09)
    make_mesh(base, output / 'vlm_board.dae', output / 'vlm_sample.jpg', 0.32, 0.24)
    with (nav / 'config/competition_interfaces.yaml').open() as stream:
        interfaces = yaml.safe_load(stream)
    interfaces.update(profile_confirmed=True, dry_run=True,
                      object_target_id=113, object_receive_max_age=0.5, object_kp=0.002)
    interfaces['dry_run_shoot_topic'] = '/aigc/competition/dry_run/shoot'
    with (output / 'interfaces.yaml').open('w') as stream:
        stream.write('# AIGC TEST WORLD ONLY; not a calibrated real-car profile.\n')
        yaml.safe_dump(interfaces, stream, allow_unicode=True, default_flow_style=False)
    shutil.copyfile(str(nav / 'config/competition_mission.example.yaml'),
                    str(output / 'mission.yaml'))
    shutil.copyfile(str(nav / 'config/task.yaml'), str(output / 'task.yaml'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-ws', default=os.environ.get('AIGC_VENDOR_WS'))
    parser.add_argument('--output', default='~/.ros/aigc_vendor/competition_sim_check')
    parser.add_argument('--cleanup', action='store_true', help='remove only added test visuals')
    args = parser.parse_args(argv)
    repo = Path(__file__).absolute().parent.parent
    output = Path(args.output).expanduser().absolute()
    if not args.vendor_ws and not args.cleanup:
        parser.error('source /tmp/aigc_vendor_env.sh or provide --vendor-ws')

    import rospy
    from geometry_msgs.msg import Pose
    from gazebo_msgs.srv import GetModelState, GetWorldProperties, DeleteModel, SpawnModel
    rospy.init_node('aigc_prepare_vendor_sim_check', anonymous=True)
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=10)
    world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    if not world.success:
        raise RuntimeError(world.status_message)
    remove = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    if args.cleanup:
        for name in MODEL_NAMES:
            if name in world.model_names:
                result = remove(name)
                if not result.success:
                    raise RuntimeError(result.status_message)
                print('Removed:', name)
        return 0

    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # Reject other maps/worlds before adding objects or enabling the test profile.
    expected = {'ar_target_0': (1.2, 0.65), 'ar_target_1': (0.55, 1.0),
                'ar_target_2': (-1.0, 0.35)}
    for name, xy in expected.items():
        if name not in world.model_names:
            raise RuntimeError('This helper requires unmodified AIGC test.world: ' + name)
        state = get_state(name, 'world')
        if (not state.success or
                math.hypot(state.pose.position.x - xy[0],
                           state.pose.position.y - xy[1]) > 0.02):
            raise RuntimeError('Unexpected test-target position: ' + name)
    robot = get_state('aigc_robot', 'world')
    if not robot.success:
        raise RuntimeError(robot.status_message)
    if (abs(robot.pose.orientation.x) > 0.01 or
            abs(robot.pose.orientation.y) > 0.01 or
            abs(robot.pose.position.z) > 0.03):
        raise RuntimeError('The AIGC test robot must be upright on the floor')
    prepare_files(repo, Path(args.vendor_ws).expanduser(), output)
    q = robot.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    targets = [
        # Place immediately in front of the corresponding existing AR texture.
        (MODEL_NAMES[0], 'template_113.dae', (1.2, 0.637, 0.15), 0.0),
        (MODEL_NAMES[1], 'marker_6.dae', (-1.0, 0.363, 0.15), math.pi),
        (MODEL_NAMES[2], 'vlm_board.dae',
         (robot.pose.position.x + 0.44*math.cos(yaw),
          robot.pose.position.y + 0.44*math.sin(yaw),
          robot.pose.position.z + 0.13), yaw - math.pi/2),
    ]
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for name, mesh, xyz, face_yaw in targets:
        if name in world.model_names:
            result = remove(name)
            if not result.success:
                raise RuntimeError(result.status_message)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        result = spawn(name, board_sdf(name, output / mesh, face_yaw), '', pose, 'world')
        if not result.success:
            raise RuntimeError('{}: {}'.format(name, result.status_message))
        print('Spawned:', name)
    print('READY: AIGC test-world profile, template=113, math AR=1, fruit AR=6.')
    print('Real shooting is disabled. Restart vendor identify.py to clear its cache.')
    print('Launch AFTER cache is 0,0 and /detect is 255:')
    print('roslaunch aigc_navigation competition_mission.launch use_sim_time:=true ' +
          'mission_file:=' + shlex.quote(str(output / 'mission.yaml')) + ' ' +
          'interfaces_file:=' + shlex.quote(str(output / 'interfaces.yaml')) + ' ' +
          'task_file:=' + shlex.quote(str(output / 'task.yaml')))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('ERROR:', str(exc), file=sys.stderr)
        sys.exit(1)
