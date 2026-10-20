#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Centralized regression for the AR task flow (round 5).

Runs WITHOUT ROS on the Windows dev machine. It registers fake ``rospy`` /
message / ``actionlib`` / ``tf`` modules plus a controllable wall/ROS clock,
then drives the REAL detection and odometry callbacks and the REAL filtering,
stable counting and pre-shoot firing gate. Positive task tests feed data
through the real ``_on_markers`` / ``_on_odom`` callbacks and run the real code
path; the key judgments are NOT stubbed to True.

Coverage (16 groups):
  0  marker patterns / committed PNG / map PGM consistent
  1  detection freshness (zero / pre-task / backward-stale / stale rejected)
  2  stable counting (same-frame no double count; A,B,A != 3)
  3  final gate (target drifted/lost -> no shot)
  4  odom freshness (freshly-received stale-header odom is not "stopped")
  5  whole-task deadline vs pulse/feedback (no success on timeout)
  6  shutdown at each stage -> interrupted / no further shot
  7  started pulse ends on interrupt / timeout / exception
  8  shot quota (prior failure -> no auto-success; prior success reused)
  9  detection id / dup / stale / non-finite rejection
  10 normal task shoots once (simulated success, hit=null)
  11 navigation regression (nav-only / retry / skip / cancel-confirm / LOST /
     interrupt)
  12 task-param validation
  13 stability reset on movement (alternating settle/move never accumulates)
  14 shoot/stopshoot publish failures -> failed, not success
  15 shutdown during cleanup -> interrupted / mission 130 STOPPED

Exit code 0 means all checks passed.
"""
import math
import os
import struct
import sys
import types
import zlib
import time as _time

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
SRC = os.path.join(ROOT, 'catkin_ws', 'src', 'aigc_navigation', 'src')
SCRIPTS = os.path.join(ROOT, 'catkin_ws', 'src', 'aigc_navigation', 'scripts')
sys.path.insert(0, SRC)
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, TOOLS)


# --------------------------------------------------------------------------
# Controllable clock and publication log
# --------------------------------------------------------------------------
class _Clock(object):
    def __init__(self):
        self.now = 0.0        # wall clock
        self.rostime = 0.0    # ROS /clock
        self.shutdown = False


CLOCK = _Clock()
PUBLISHED = []          # list of (topic, msg)
SERVER_CONNECTED = True
TF_AVAILABLE = True
SLEEP_HOOKS = []        # callables run on every time.sleep
GOAL_RESULTS = []       # queue of GoalStatus values consumed by send_goal


def fake_sleep(seconds):
    CLOCK.now += seconds
    CLOCK.rostime += seconds
    for hook in list(SLEEP_HOOKS):
        hook()


# --------------------------------------------------------------------------
# Fake ROS / message / actionlib / tf modules
# --------------------------------------------------------------------------
def _mod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


def _pkg(name):
    m = types.ModuleType(name)
    m.__path__ = []
    sys.modules[name] = m
    return m


class Time(object):
    def __init__(self, secs=0.0, nsecs=0):
        self._secs = float(secs) + nsecs / 1e9

    def to_sec(self):
        return self._secs

    def to_nsec(self):
        return int(self._secs * 1e9)

    def __sub__(self, other):
        return Duration(self._secs - other.to_sec())

    def __gt__(self, other):
        return self._secs > other.to_sec()

    def __lt__(self, other):
        return self._secs < other.to_sec()

    def __ge__(self, other):
        return self._secs >= other.to_sec()

    def __le__(self, other):
        return self._secs <= other.to_sec()

    def __eq__(self, other):
        return self._secs == other.to_sec()

    def __ne__(self, other):
        return self._secs != other.to_sec()

    def __hash__(self):
        return hash(self._secs)

    @staticmethod
    def now():
        return Time(CLOCK.rostime)


class Duration(Time):
    pass


class Header(object):
    def __init__(self):
        self.seq = 0
        self.stamp = Time(0.0)
        self.frame_id = ''


class Vector3(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Quaternion(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class Point(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Pose(object):
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class PoseStamped(object):
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class Twist(object):
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class TwistStamped(object):
    def __init__(self):
        self.header = Header()
        self.twist = Twist()


class PoseWithCovariance(object):
    def __init__(self):
        self.pose = Pose()
        self.covariance = [0.0] * 36


class TwistWithCovariance(object):
    def __init__(self):
        self.twist = Twist()
        self.covariance = [0.0] * 36


class Odometry(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ''
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()


class AlvarMarker(object):
    def __init__(self):
        self.header = Header()
        self.id = 0
        self.pose = PoseStamped()   # ar_track_alvar uses geometry_msgs/PoseStamped
        self.confidence = 0


class AlvarMarkers(object):
    def __init__(self):
        self.header = Header()
        self.markers = []


class String(object):
    def __init__(self, data=''):
        self.data = data


class MoveBaseGoal(object):
    def __init__(self):
        self.target_pose = PoseStamped()


class MoveBaseAction(object):
    pass


class GoalStatus(object):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


class Publisher(object):
    def __init__(self, name, msg_class, queue_size=1):
        self.name = name
        self.msg_class = msg_class

    def publish(self, msg):
        PUBLISHED.append((self.name, msg))


class Subscriber(object):
    def __init__(self, name, msg_class, callback=None):
        self.name = name
        self.msg_class = msg_class
        self.callback = callback


class SimpleActionClient(object):
    def __init__(self, name, ActionSpec):
        self.name = name
        self.spec = ActionSpec
        self.state = GoalStatus.PENDING
        self.sent = []
        self.cancels = []

    def wait_for_server(self):
        return SERVER_CONNECTED

    def send_goal(self, goal):
        self.sent.append(goal)
        if GOAL_RESULTS:
            self.state = GOAL_RESULTS.pop(0)
        else:
            self.state = GoalStatus.ACTIVE

    def get_state(self):
        return self.state

    def cancel_goal(self):
        self.cancels.append(self.state)


class TransformListener(object):
    def __init__(self, *args, **kwargs):
        self.can_transform = TF_AVAILABLE

    def canTransform(self, target, source, t):
        return TF_AVAILABLE


# -- rospy ---------------------------------------------------------------
_rospy = _mod('rospy')
_rospy.Time = Time
_rospy.Duration = Duration
_rospy.Publisher = Publisher
_rospy.Subscriber = Subscriber
_rospy.loginfo = lambda *a, **k: None
_rospy.logwarn = lambda *a, **k: None
_rospy.logerr = lambda *a, **k: None
_rospy.logdebug = lambda *a, **k: None
_rospy.init_node = lambda *a, **k: None
_rospy.on_shutdown = lambda *a, **k: None
_rospy.sleep = fake_sleep
_rospy.is_shutdown = lambda: CLOCK.shutdown
_rospy.get_rostime = lambda: Time(CLOCK.rostime)
_rospy.get_param = lambda name, default=None: default
_rospy.has_param = lambda name: False
_rospy.set_param = lambda *a, **k: None

# -- std_msgs ------------------------------------------------------------
_std_msgs = _pkg('std_msgs')
_std_msgs_msg = _mod('std_msgs.msg')
_std_msgs_msg.String = String
_std_msgs.msg = _std_msgs_msg

# -- geometry_msgs --------------------------------------------------------
_geom = _pkg('geometry_msgs')
_geom_msg = _mod('geometry_msgs.msg')
_geom_msg.Twist = Twist
_geom_msg.Quaternion = Quaternion
_geom_msg.Point = Point
_geom_msg.Pose = Pose
_geom_msg.PoseStamped = PoseStamped
_geom_msg.TwistStamped = TwistStamped
_geom_msg.Vector3 = Vector3
_geom.msg = _geom_msg

# -- nav_msgs -------------------------------------------------------------
_nav = _pkg('nav_msgs')
_nav_msg = _mod('nav_msgs.msg')
_nav_msg.Odometry = Odometry
_nav.msg = _nav_msg

# -- ar_track_alvar_msgs --------------------------------------------------
_ata = _pkg('ar_track_alvar_msgs')
_ata_msg = _mod('ar_track_alvar_msgs.msg')
_ata_msg.AlvarMarker = AlvarMarker
_ata_msg.AlvarMarkers = AlvarMarkers
_ata.msg = _ata_msg

# -- move_base_msgs -------------------------------------------------------
_mbm = _pkg('move_base_msgs')
_mbm_msg = _mod('move_base_msgs.msg')
_mbm_msg.MoveBaseAction = MoveBaseAction
_mbm_msg.MoveBaseGoal = MoveBaseGoal
_mbm_msg.MoveBaseActionResult = MoveBaseAction
_mbm.msg = _mbm_msg

# -- actionlib ------------------------------------------------------------
_actionlib = _mod('actionlib')
_actionlib.GoalStatus = GoalStatus
_actionlib.SimpleActionClient = SimpleActionClient

# -- tf --------------------------------------------------------------------
_tf = _mod('tf')
_tf.TransformListener = TransformListener

# Patch the shared wall clock and time.sleep.
import aigc_navigation.navigation_client as _nc            # noqa: E402
_nc._wall_clock = lambda: CLOCK.now
_time.sleep = fake_sleep

from aigc_navigation.sim_shooter import SimShooter          # noqa: E402
from aigc_navigation.task_executor import (                 # noqa: E402
    TaskExecutor, TASK_SUCCESS, TASK_FAILED, TASK_TIMEOUT, TASK_INTERRUPTED)
import mission_manager                                      # noqa: E402


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
FAILURES = []
CHECKS = [0]


def check(cond, label):
    CHECKS[0] += 1
    if cond:
        print('  ok   %s' % label)
    else:
        print('  FAIL %s' % label)
        FAILURES.append(label)


def reset():
    CLOCK.now = 0.0
    CLOCK.rostime = 0.0
    CLOCK.shutdown = False
    del PUBLISHED[:]
    del SLEEP_HOOKS[:]
    del GOAL_RESULTS[:]
    SERVER_CONNECTED = True
    TF_AVAILABLE = True


def published_data(topic):
    return [m.data for t, m in PUBLISHED if t == topic and hasattr(m, 'data')]


def task_cfg(**over):
    cfg = {
        'output_frame': 'camera_optical_frame',
        'ar_marker_topic': '/ar_pose_marker',
        'odom_topic': '/odom',
        'control_rate': 20.0,
        'kp': 1.0,
        'max_angular_rate': 0.35,
        'search_rate': 0.15,
        'align_tolerance': 0.035,
        'stable_frames': 3,
        'detection_max_age': 0.3,
        'receive_max_age': 0.5,
        'task_timeout': 20.0,
        'settle_timeout': 3.0,
        'odom_max_age': 0.5,
        'odom_max_linear': 0.05,
        'odom_max_angular': 0.05,
        'stop_publish_count': 3,
        'stop_publish_interval': 0.05,
        'shooter': {'topic': '/shoot', 'pulse_duration': 0.2, 'delay': 0.1,
                    'success': True, 'failure_reason': 'simulated failure'},
    }
    cfg.update(over)
    return cfg


def make_odom(stamp_sec, lin=0.0, ang=0.0):
    o = Odometry()
    o.header.stamp = Time(stamp_sec)
    o.header.frame_id = 'odom'
    o.twist.twist.linear.x = lin
    o.twist.twist.linear.y = 0.0
    o.twist.twist.angular.z = ang
    return o


def make_marker(marker_id, stamp_sec, x=0.0, y=0.0, z=1.0,
                frame='camera_optical_frame'):
    mk = AlvarMarker()
    mk.header.stamp = Time(stamp_sec)
    mk.header.frame_id = frame
    mk.id = marker_id
    mk.pose.pose.position.x = x
    mk.pose.pose.position.y = y
    mk.pose.pose.position.z = z
    return mk


def make_markers(markers):
    m = AlvarMarkers()
    m.header.stamp = Time(CLOCK.rostime)
    m.markers = markers
    return m


def make_executor(cfg=None):
    return TaskExecutor(cfg or task_cfg())


def read_png(path):
    data = open(path, 'rb').read()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    pos = 8
    width = height = None
    idat = b''
    while pos < len(data):
        length = struct.unpack('>I', data[pos:pos + 4])[0]
        typ = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        if typ == b'IHDR':
            (width, height, bitdepth, colortype, comp, filt, inter) = \
                struct.unpack('>IIBBBBB', chunk)
        elif typ == b'IDAT':
            idat += chunk
        elif typ == b'IEND':
            break
        pos += 12 + length
    raw = zlib.decompress(idat)
    rows = []
    stride = width  # grayscale 8-bit, 1 byte/pixel
    for r in range(height):
        line = raw[r * (stride + 1) + 1:(r + 1) * (stride + 1)]
        rows.append(list(line))
    return width, height, rows


def read_pgm(path):
    data = open(path, 'rb').read()
    assert data[:2] == b'P5'
    idx = 0
    for _ in range(3):
        idx = data.index(b'\n', idx) + 1
    parts = data[2:idx].decode('ascii').split()
    w, h, mx = int(parts[0]), int(parts[1]), int(parts[2])
    pix = data[idx:idx + w * h]
    return w, h, mx, pix


# --------------------------------------------------------------------------
# Group 0: patterns / committed PNG / map PGM consistency
# --------------------------------------------------------------------------
def test_static_data():
    print('[0] patterns / PNG / map consistency')
    import generate_test_markers as gen

    for mid, pat in gen.PATTERNS.items():
        ok = (isinstance(pat, list) and len(pat) == 9 and
              all(isinstance(r, str) and len(r) == 9 and
                  all(c in '01' for c in r) for r in pat))
        check(ok, 'pattern %d is a 9x9 binary grid' % mid)

    tex_dir = os.path.join(ROOT, 'catkin_ws', 'src', 'aigc_sim',
                           'materials', 'textures')
    for mid, pat in gen.PATTERNS.items():
        w, h, rows = read_png(os.path.join(tex_dir, 'marker_%d.png' % mid))
        expected = gen.marker_rows(pat)
        check((w, h) == (180, 180) and rows == expected,
              'marker_%d.png matches its pattern' % mid)

    map_path = os.path.join(ROOT, 'catkin_ws', 'src', 'aigc_navigation',
                            'maps', 'test.pgm')
    w, h, mx, pix = read_pgm(map_path)
    occupied = sum(1 for b in pix if b < 90)
    free = sum(1 for b in pix if b > 205)
    check((w, h, mx) == (200, 200, 255) and len(pix) == 40000,
          'test.pgm is a valid 200x200 P5 map')
    check(occupied == 4160 and free == 35840,
          'test.pgm occupancy matches the recorded baseline')


# --------------------------------------------------------------------------
# Group 1: detection freshness (zero / pre-task / backward / stale)
# --------------------------------------------------------------------------
def test_detection_freshness():
    print('[1] detection freshness')
    reset()
    CLOCK.rostime = 100.0
    e = make_executor()
    e._task_start_ros = Time(0.0)
    e._markers_stamp = CLOCK.now

    # valid
    e._markers = make_markers([make_marker(0, 99.9, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is not None, 'valid marker accepted')

    # zero stamp
    e._markers = make_markers([make_marker(0, 0.0, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is None, 'zero-stamp marker rejected')

    # pre-task / backward image (captured before task start)
    e._task_start_ros = Time(100.0)
    e._markers = make_markers([make_marker(0, 99.9, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is None,
          'pre-task (backward) image rejected')

    # stale image age
    e._task_start_ros = Time(0.0)
    e._markers = make_markers([make_marker(0, 99.0, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is None, 'stale image rejected')


# --------------------------------------------------------------------------
# Group 2: stable counting (same-frame no double count; A,B,A != 3)
# --------------------------------------------------------------------------
def _run_align_sequence(e, stamps, deadline_after=1.0):
    """Feed odom + scripted marker stamps once per align-loop iteration."""
    seq = list(stamps)

    def scripted_sleep(seconds):
        CLOCK.now += seconds
        e._on_odom(make_odom(CLOCK.rostime))
        if seq:
            s = seq.pop(0)
            e._on_markers(make_markers([make_marker(0, s, x=0.0, z=1.0)]))
        else:
            e._on_markers(make_markers([]))

    e._interruptible_sleep = scripted_sleep
    e._task_start_ros = Time(0.0)
    return e._search_align(0, deadline=CLOCK.now + deadline_after)


def test_stable_counting():
    print('[2] stable counting')
    reset()
    CLOCK.rostime = 100.0

    e = make_executor()
    res = _run_align_sequence(e, [99.7, 99.8, 99.9, 99.9], deadline_after=1.0)
    check(res == TASK_SUCCESS, 'strictly-increasing stamps reach success')

    e = make_executor()
    res = _run_align_sequence(e, [99.8] * 20, deadline_after=1.0)
    check(res == TASK_TIMEOUT, 'same-frame re-read never reaches 3')

    e = make_executor()
    res = _run_align_sequence(e, [99.8, 99.9, 99.8, 99.8, 99.8, 99.8],
                              deadline_after=1.0)
    check(res == TASK_TIMEOUT, 'A,B,A sequence never reaches 3')


# --------------------------------------------------------------------------
# Group 3: final gate (target drifted/lost -> no shot)
# --------------------------------------------------------------------------
def test_final_gate():
    print('[3] final gate before shoot')
    reset()
    CLOCK.rostime = 100.0
    e = make_executor()
    e._task_start_ros = Time(100.0)
    e._markers_stamp = CLOCK.now
    e._odom_stamp = CLOCK.now
    e._odom = make_odom(CLOCK.rostime)
    deadline = CLOCK.now + 10.0

    # drifted (off tolerance)
    e._markers = make_markers([make_marker(0, CLOCK.rostime, x=1.0, z=1.0)])
    res = e._final_gate_and_shoot('r', 'task_1', 0, True, deadline)
    check(res['status'] == TASK_FAILED and res['shot'] is None,
          'drifted target -> failed, no shot')
    check(published_data('/shoot') == [], 'no shoot command published')

    # lost (no marker)
    e._markers = make_markers([])
    res = e._final_gate_and_shoot('r', 'task_1', 0, True, deadline)
    check(res['status'] == TASK_FAILED and res['shot'] is None,
          'lost target -> failed, no shot')

    # not settled
    e._markers = make_markers([make_marker(0, CLOCK.rostime, x=0.0, z=1.0)])
    e._odom = make_odom(CLOCK.rostime, lin=0.5)
    res = e._final_gate_and_shoot('r', 'task_1', 0, True, deadline)
    check(res['status'] == TASK_FAILED and res['shot'] is None,
          'moving robot -> failed, no shot')


# --------------------------------------------------------------------------
# Group 4: odom freshness (stale header, freshly received)
# --------------------------------------------------------------------------
def test_odom_freshness():
    print('[4] odom freshness')
    reset()
    CLOCK.rostime = 100.0
    e = make_executor()
    e._odom_stamp = CLOCK.now

    e._odom = make_odom(99.0)  # header age 1.0 > odom_max_age 0.5
    check(not e._odom_stable(),
          'freshly-received stale-header odom is not "stopped"')

    e._odom = make_odom(0.0)  # zero header stamp
    check(not e._odom_stable(), 'zero-stamp odom is not "stopped"')

    e._odom = make_odom(CLOCK.rostime, lin=0.0, ang=0.0)
    check(e._odom_stable(), 'fresh settled odom is "stopped"')

    e._odom = make_odom(CLOCK.rostime, lin=0.1, ang=0.0)
    check(not e._odom_stable(), 'moving odom is not "stopped"')


# --------------------------------------------------------------------------
# Group 5: whole-task deadline vs pulse/feedback
# --------------------------------------------------------------------------
def test_deadline_vs_pulse():
    print('[5] whole-task deadline vs pulse/feedback')
    reset()
    CLOCK.now = 100.0
    ss = SimShooter({'topic': '/shoot', 'pulse_duration': 0.2, 'delay': 0.1,
                     'success': True, 'failure_reason': 'x'})

    res = ss.shoot('r', 'w', 0, 'sid', deadline=CLOCK.now + 0.05)
    check(res['status'] == 'timeout' and not res['success'],
          'pulse exceeding deadline -> timeout, not success')
    check('stopshoot' in published_data('/shoot'),
          'pulse ended (stopshoot) on deadline timeout')

    del PUBLISHED[:]
    res = ss.shoot('r', 'w', 0, 'sid', deadline=CLOCK.now)  # already expired
    check(res['status'] == 'timeout' and not res['success'],
          'already-expired deadline -> timeout, no success')
    check(published_data('/shoot') == [], 'no shoot command when expired')


# --------------------------------------------------------------------------
# Group 6: shutdown at each stage -> interrupted / no further shot
# --------------------------------------------------------------------------
def test_shutdown_stages():
    print('[6] shutdown at each stage')
    reset()
    CLOCK.now = 100.0
    CLOCK.rostime = 100.0

    # at start (settle stage)
    CLOCK.shutdown = True
    e = make_executor()
    res = e.execute('r', 'task_1', 0, allow_shot=True)
    check(res['status'] == TASK_INTERRUPTED, 'shutdown at start -> interrupted')
    check(published_data('/shoot') == [], 'no shoot on shutdown at start')
    CLOCK.shutdown = False

    # during search
    e = make_executor()
    e._task_start_ros = Time(100.0)

    def search_shutdown_sleep(seconds):
        CLOCK.now += seconds
        e._on_odom(make_odom(CLOCK.rostime))
        e._on_markers(make_markers([make_marker(0, CLOCK.rostime, x=0.0,
                                                z=1.0)]))
        CLOCK.shutdown = True

    e._interruptible_sleep = search_shutdown_sleep
    res = e._search_align(0, deadline=CLOCK.now + 10.0)
    check(res == TASK_INTERRUPTED, 'shutdown during search -> interrupted')
    check(published_data('/shoot') == [], 'no shoot on shutdown during search')
    CLOCK.shutdown = False

    # during shoot
    ss = SimShooter({'topic': '/shoot', 'pulse_duration': 0.2, 'delay': 0.1,
                     'success': True, 'failure_reason': 'x'})

    def shoot_shutdown_hook():
        CLOCK.shutdown = True

    SLEEP_HOOKS.append(shoot_shutdown_hook)
    res = ss.shoot('r', 'w', 0, 'sid', deadline=CLOCK.now + 10.0)
    SLEEP_HOOKS[:] = []
    check(res['status'] == 'interrupted' and not res['success'],
          'shutdown during shoot -> interrupted, not success')
    check('stopshoot' in published_data('/shoot'), 'pulse ended on interrupt')
    CLOCK.shutdown = False


# --------------------------------------------------------------------------
# Group 7: started pulse ends on interrupt / timeout / exception
# --------------------------------------------------------------------------
def test_pulse_cleanup():
    print('[7] started pulse ends on interrupt/timeout/exception')
    reset()
    CLOCK.now = 100.0

    def fresh_shooter():
        return SimShooter({'topic': '/shoot', 'pulse_duration': 0.2,
                           'delay': 0.1, 'success': True,
                           'failure_reason': 'x'})

    # timeout path already covered in group 5; here check exception path.
    ss = fresh_shooter()

    def boom(*a, **k):
        raise RuntimeError('boom')

    ss._wait = boom
    try:
        ss.shoot('r', 'w', 0, 'sid', deadline=CLOCK.now + 10.0)
        raised = False
    except RuntimeError:
        raised = True
    check(raised, 'exception during pulse propagates')
    check(ss._pulse_active is False and 'stopshoot' in published_data('/shoot'),
          'pulse ended (stopshoot) via try/finally on exception')


# --------------------------------------------------------------------------
# Group 8: shot quota (prior failure vs prior success)
# --------------------------------------------------------------------------
def _drive_flow(e, allow_shot):
    """Run execute() with a real-callback feed of aligned + settled data."""
    e._task_start_ros = Time(CLOCK.rostime)

    def feed():
        e._on_odom(make_odom(CLOCK.rostime))
        e._on_markers(make_markers([make_marker(0, CLOCK.rostime, x=0.0,
                                                z=1.0)]))

    SLEEP_HOOKS.append(feed)
    res = e.execute('r1', 'task_1', 0, allow_shot=allow_shot)
    SLEEP_HOOKS[:] = []
    return res


def test_shot_quota():
    print('[8] shot quota')
    # prior simulated failure -> next call must not auto-succeed
    reset()
    CLOCK.now = 100.0
    CLOCK.rostime = 100.0
    e = make_executor(task_cfg(shooter={'topic': '/shoot', 'pulse_duration':
                                        0.2, 'delay': 0.1, 'success': False,
                                        'failure_reason': 'simulated failure'}))
    res1 = _drive_flow(e, allow_shot=True)
    check(res1['status'] == TASK_FAILED and res1['shot'] is not None,
          'first shot with simulated failure -> failed (shot recorded)')
    res2 = _drive_flow(e, allow_shot=False)
    check(res2['status'] == TASK_FAILED and res2['shot'] is None,
          'prior failure is not rewritten to success (no auto-success)')
    check(published_data('/shoot').count('shoot') == 1, 'still exactly one shot')

    # prior success -> reuse
    reset()
    CLOCK.now = 100.0
    CLOCK.rostime = 100.0
    e = make_executor()
    res1 = _drive_flow(e, allow_shot=True)
    check(res1['status'] == TASK_SUCCESS and res1['shot'] is not None,
          'first shot succeeds')
    res2 = _drive_flow(e, allow_shot=False)
    check(res2['status'] == TASK_SUCCESS and res2['shot'] is None,
          'prior success reused without a second shot')
    check(published_data('/shoot').count('shoot') == 1,
          'quota: at most one shot per waypoint')


# --------------------------------------------------------------------------
# Group 9: id / dup / stale / non-finite rejection
# --------------------------------------------------------------------------
def test_detection_rejections():
    print('[9] detection id/dup/stale/non-finite rejection')
    reset()
    CLOCK.rostime = 100.0
    e = make_executor()
    e._task_start_ros = Time(0.0)
    e._markers_stamp = CLOCK.now

    e._markers = make_markers([make_marker(5, 99.9, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is None, 'wrong id rejected')

    e._markers = make_markers([make_marker(0, 99.9, x=0.0, z=1.0),
                               make_marker(0, 99.9, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is None, 'duplicate id rejected')

    e._markers = make_markers([make_marker(0, 99.0, x=0.0, z=1.0)])
    check(e._latest_valid_detection(0) is None, 'stale rejected')

    mk = make_marker(0, 99.9, x=0.0, z=1.0)
    mk.pose.pose.position.x = float('nan')
    e._markers = make_markers([mk])
    check(e._latest_valid_detection(0) is None, 'non-finite coordinate rejected')


# --------------------------------------------------------------------------
# Group 10: normal task shoots once (simulated success, hit=null)
# --------------------------------------------------------------------------
def test_normal_flow():
    print('[10] normal task shoots once')
    reset()
    CLOCK.now = 100.0
    CLOCK.rostime = 100.0
    e = make_executor()
    res = _drive_flow(e, allow_shot=True)
    check(res['status'] == TASK_SUCCESS, 'normal task succeeds')
    shot = res['shot']
    check(shot is not None and shot['status'] == 'success',
          'shot recorded as simulated success')
    check(shot['hit'] is None and shot['backend'] == 'simulated',
          'feedback is backend=simulated / hit=null')
    check(published_data('/shoot').count('shoot') == 1,
          'exactly one shoot command published')
    check('stopshoot' in published_data('/shoot'), 'pulse ended normally')


# --------------------------------------------------------------------------
# Group 11: navigation + task-glue regression
# --------------------------------------------------------------------------
def _mission_cfg(**over):
    cfg = {
        'frame_id': 'map',
        'ready_timeout': 5.0,
        'server_timeout': 2.0,
        'goal_timeout': 2.0,
        'cancel_timeout': 1.0,
        'max_retries': 1,
        'failure_policy': 'stop',
        'dwell_time': 0.5,
        'execute_tasks': False,
        'waypoints': [],
        'log_dir': '',
    }
    cfg.update(over)
    return cfg


class FakeExecutor(object):
    def __init__(self, result):
        self.result = result
        self.calls = []
        self.cleaned = False

    def execute(self, run_id, waypoint, target_id, allow_shot=True):
        self.calls.append((waypoint, allow_shot))
        return dict(self.result)

    def cleanup(self):
        self.cleaned = True


def test_navigation_regression():
    print('[11] navigation / task-glue regression')
    reset()
    CLOCK.rostime = 100.0

    def wp(name, kind, x=0.0, y=0.0, yaw=0.0, **extra):
        d = {'name': name, 'kind': kind, 'x': x, 'y': y, 'yaw': yaw}
        d.update(extra)
        return d

    # nav-only: transit + finish complete with exit 0
    GOAL_RESULTS.extend([GoalStatus.SUCCEEDED, GoalStatus.SUCCEEDED])
    runner = mission_manager.MissionRunner(_mission_cfg(
        waypoints=[wp('t', 'transit'), wp('f', 'finish')]))
    runner.client.wait_for_server = lambda timeout: True
    code, status, _ = runner.run()
    check((code, status) == (0, mission_manager.STATUS_COMPLETED),
          'nav-only run completes (exit 0 / COMPLETED)')

    # retry: aborted once then succeeded
    del GOAL_RESULTS[:]
    GOAL_RESULTS.extend([GoalStatus.ABORTED, GoalStatus.SUCCEEDED])
    runner = mission_manager.MissionRunner(_mission_cfg())
    res = runner.navigate_to(wp('t', 'transit'))
    check(res['status'] == mission_manager.WP_SUCCESS and res['attempts'] == 2,
          'nav retries after aborted and succeeds')

    # cancel unconfirmed: goal never reaches a terminal state
    del GOAL_RESULTS[:]
    GOAL_RESULTS.extend([GoalStatus.ACTIVE])
    runner = mission_manager.MissionRunner(_mission_cfg())
    res = runner.navigate_to(wp('t', 'transit'))
    check(res['status'] == mission_manager.OUT_CANCEL_UNCONFIRMED,
          'unconfirmed cancel -> OUT_CANCEL_UNCONFIRMED')

    # LOST
    del GOAL_RESULTS[:]
    GOAL_RESULTS.extend([GoalStatus.LOST])
    runner = mission_manager.MissionRunner(_mission_cfg())
    res = runner.navigate_to(wp('t', 'transit'))
    check(res['status'] == mission_manager.OUT_LOST, 'LOST -> OUT_LOST')

    # task-glue: skip policy on failed task
    runner = mission_manager.MissionRunner(_mission_cfg(
        execute_tasks=True, failure_policy='skip'))
    runner.task_cfg = task_cfg()
    runner.task_executor = FakeExecutor({'status': 'failed', 'aligned': False,
                                         'shot': None, 'reason': 'did not settle'})
    ret = runner._run_task_waypoint(wp('task_1', 'task', target_id=0), 1)
    check(ret is None and runner.skipped_any,
          'task failure + skip policy -> continue, marked skipped')

    # task-glue: stop policy on failed task
    runner = mission_manager.MissionRunner(_mission_cfg(
        execute_tasks=True, failure_policy='stop'))
    runner.task_cfg = task_cfg()
    runner.task_executor = FakeExecutor({'status': 'failed', 'aligned': False,
                                         'shot': None, 'reason': 'timeout'})
    ret = runner._run_task_waypoint(wp('task_1', 'task', target_id=0), 1)
    check(ret is not None and ret[0] == 2 and ret[1] == mission_manager.STATUS_FAILED,
          'task failure + stop policy -> exit 2 / FAILED')

    # task-glue: interrupted -> exit 130 / STOPPED
    runner = mission_manager.MissionRunner(_mission_cfg(
        execute_tasks=True, failure_policy='skip'))
    runner.task_cfg = task_cfg()
    runner.task_executor = FakeExecutor({'status': 'interrupted',
                                         'aligned': False, 'shot': None,
                                         'reason': 'shutdown'})
    ret = runner._run_task_waypoint(wp('task_1', 'task', target_id=0), 1)
    check(ret is not None and ret[0] == 130 and ret[1] == mission_manager.STATUS_STOPPED,
          'task interrupt -> exit 130 / STOPPED')


# --------------------------------------------------------------------------
# Group 12: task-param validation
# --------------------------------------------------------------------------
def test_param_validation():
    print('[12] task-param validation')
    reset()
    good = task_cfg()
    check(mission_manager.validate_task_config(good) == [],
          'valid task config accepted')

    bad = task_cfg(shooter={'topic': '/shoot', 'pulse_duration': 0.2,
                            'delay': 0.1, 'success': 'yes',
                            'failure_reason': 'x'})
    check(any('shooter.success' in m for m in
              mission_manager.validate_task_config(bad)),
          'non-bool shooter.success rejected')

    bad = task_cfg(stable_frames=True)
    check(any('stable_frames' in m for m in
              mission_manager.validate_task_config(bad)),
          'bool stable_frames rejected')

    bad = task_cfg(stable_frames=0)
    check(any('stable_frames' in m for m in
              mission_manager.validate_task_config(bad)),
          'non-positive stable_frames rejected')

    bad = task_cfg(search_rate=1.0)  # > max_angular_rate 0.35
    check(any('search_rate' in m for m in
              mission_manager.validate_task_config(bad)),
          'search_rate > max_angular_rate rejected')

    bad = task_cfg(shooter={'topic': '/shoot', 'pulse_duration': 0.0,
                            'delay': 0.1, 'success': True,
                            'failure_reason': 'x'})
    check(any('pulse_duration' in m for m in
              mission_manager.validate_task_config(bad)),
          'non-positive pulse_duration rejected')

    bad = task_cfg(output_frame='')
    check(any('output_frame' in m for m in
              mission_manager.validate_task_config(bad)),
          'empty output_frame rejected')

    # mission-level: execute_tasks must be a boolean
    cfg = _mission_cfg()
    cfg['execute_tasks'] = 'yes'
    check(any('execute_tasks' in m for m in
              mission_manager.validate_config(cfg)),
          'non-bool execute_tasks rejected')


# --------------------------------------------------------------------------
# Group 13: stability reset on movement (alternating settle/move)
# --------------------------------------------------------------------------
def _run_script(e, script, deadline_after):
    """Feed a scripted (marker_stamp, odom_linear_x) pair per align iteration."""
    seq = list(script)

    def scripted_sleep(seconds):
        CLOCK.now += seconds
        if seq:
            stamp, lin = seq.pop(0)
            e._on_odom(make_odom(CLOCK.rostime, lin=lin))
            e._on_markers(make_markers([make_marker(0, stamp, x=0.0, z=1.0)]))
        else:
            e._on_odom(make_odom(CLOCK.rostime))
            e._on_markers(make_markers([]))

    e._interruptible_sleep = scripted_sleep
    e._task_start_ros = Time(0.0)
    return e._search_align(0, deadline=CLOCK.now + deadline_after)


def test_stability_reset():
    print('[13] stability reset on movement')
    reset()
    CLOCK.rostime = 100.0

    # Alternating settled/moving must NOT accumulate to stable_frames=3.
    e = make_executor()
    res = _run_script(e, [
        (99.70, 0.0),   # settled -> 1
        (99.72, 0.5),   # moving  -> reset to 0
        (99.74, 0.0),   # settled -> 1
        (99.76, 0.5),   # moving  -> reset to 0
        (99.78, 0.0),   # settled -> 1
    ], deadline_after=1.0)
    check(res == TASK_TIMEOUT,
          'alternating settle/move never accumulates success')

    # After a move resets the run, three consecutive settled NEW frames allow a
    # shot. (Fresh clock: the timeout's _stop() above advanced the ROS clock.)
    reset()
    CLOCK.rostime = 100.0
    e = make_executor()
    res = _run_script(e, [
        (99.70, 0.0),   # settled -> 1
        (99.72, 0.5),   # moving  -> reset to 0
        (99.74, 0.0),   # settled -> 1
        (99.76, 0.0),   # settled -> 2
        (99.78, 0.0),   # settled -> 3 -> success
    ], deadline_after=1.0)
    check(res == TASK_SUCCESS,
          'consecutive settled new frames after a move allow shooting')


# --------------------------------------------------------------------------
# Group 14: shoot/stopshoot publish failures -> failed, not success
# --------------------------------------------------------------------------
def test_publish_failure():
    print('[14] shoot/stopshoot publish failures')
    reset()
    CLOCK.now = 100.0

    # 'shoot' publish always raises.
    ss = SimShooter({'topic': '/shoot', 'pulse_duration': 0.2, 'delay': 0.1,
                     'success': True, 'failure_reason': 'x'})

    def boom(_msg):
        raise RuntimeError('publish down')

    ss._pub.publish = boom
    res = ss.shoot('r', 'w', 0, 'sid', deadline=CLOCK.now + 10.0)
    check(res['status'] == 'failed' and not res['success'],
          'shoot publish failure -> failed, not success')
    check('shoot' in res['reason'],
          'shoot publish failure recorded in reason')
    check(published_data('/shoot') == [], 'no command reached the wire')
    check(ss._pulse_active is False, 'no pulse started on start failure')

    # 'stopshoot' publish raises: start succeeds, stop fails.
    reset()
    CLOCK.now = 100.0
    ss = SimShooter({'topic': '/shoot', 'pulse_duration': 0.2, 'delay': 0.1,
                     'success': True, 'failure_reason': 'x'})
    real_publish = ss._pub.publish

    def fail_stop(msg):
        if getattr(msg, 'data', '') == 'stopshoot':
            raise RuntimeError('stop down')
        return real_publish(msg)

    ss._pub.publish = fail_stop
    res = ss.shoot('r', 'w', 0, 'sid', deadline=CLOCK.now + 10.0)
    check(res['status'] == 'failed' and not res['success'],
          'stopshoot publish failure -> failed, not success')
    check('stopshoot' in res['reason'],
          'stopshoot publish failure recorded in reason')
    check(ss._pulse_active is False, 'pulse inactive after failed stop')
    check(published_data('/shoot').count('shoot') == 1,
          'exactly one shoot command reached the wire')


# --------------------------------------------------------------------------
# Group 15: shutdown during cleanup -> interrupted / mission 130 STOPPED
# --------------------------------------------------------------------------
def test_cleanup_interrupt():
    print('[15] shutdown during cleanup')
    reset()
    CLOCK.now = 100.0
    CLOCK.rostime = 100.0

    def setup_executor():
        e = make_executor()
        # Settle runs on the clock only (no hooks), so it times out to
        # 'not_settled'; shutdown then lands inside the finally _stop().
        def fast(seconds):
            CLOCK.now += seconds
        e._interruptible_sleep = fast
        return e

    # (a) task-level: settle fails, shutdown lands during the finally _stop().
    e = setup_executor()
    SLEEP_HOOKS.append(lambda: setattr(CLOCK, 'shutdown', True))
    res = e.execute('r', 'task_1', 0, allow_shot=True)
    SLEEP_HOOKS[:] = []
    check(res['status'] == TASK_INTERRUPTED,
          'task returns interrupted when shutdown lands during cleanup')
    check(published_data('/shoot') == [], 'no shoot on cleanup shutdown')
    CLOCK.shutdown = False

    # (b) mission-level: that interrupted result exits 130 / STOPPED even under
    #     failure_policy=skip.
    runner = mission_manager.MissionRunner(_mission_cfg(
        execute_tasks=True, failure_policy='skip'))
    runner.task_cfg = task_cfg()
    runner.task_executor = setup_executor()
    SLEEP_HOOKS.append(lambda: setattr(CLOCK, 'shutdown', True))
    wp = {'name': 'task_1', 'kind': 'task', 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
          'target_id': 0}
    ret = runner._run_task_waypoint(wp, 1)
    SLEEP_HOOKS[:] = []
    CLOCK.shutdown = False
    check(ret is not None and ret[0] == 130 and
          ret[1] == mission_manager.STATUS_STOPPED,
          'cleanup shutdown -> mission exits 130 / STOPPED (beats skip)')


# --------------------------------------------------------------------------
def main():
    test_static_data()
    test_detection_freshness()
    test_stable_counting()
    test_final_gate()
    test_odom_freshness()
    test_deadline_vs_pulse()
    test_shutdown_stages()
    test_pulse_cleanup()
    test_shot_quota()
    test_detection_rejections()
    test_normal_flow()
    test_navigation_regression()
    test_param_validation()
    test_stability_reset()
    test_publish_failure()
    test_cleanup_interrupt()

    print('')
    if FAILURES:
        print('%d/%d checks FAILED:' % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print('  - %s' % f)
        return 1
    print('ALL %d CHECKS PASSED' % CHECKS[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
