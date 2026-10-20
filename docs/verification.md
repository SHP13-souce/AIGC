# 验证记录

本文件按轮次记录验证情况：第一轮（仿真底座）与第二轮（单点导航）。每轮在 Windows 上完成静态检查；Ubuntu 上的编译与运行验证留待后续执行。

## 第一轮：仿真底座

本轮范围：机器人模型、测试场地、仿真启动文件、传感器配置、运行说明。

起始 commit：`a7fd986bd06be67a02fbde3d73cfe0256edc1fb9`

---

### A. Windows 已实际执行的检查

在 Windows（无 ROS/Gazebo）上执行，仅使用已安装的 Python 3.13 标准库。

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 所有 XML 文件语法 | Python `xml.etree.ElementTree` 逐个解析 | 通过（见下方命令） |
| 2 | 包名 / 依赖 / CMake | 人工核对 package.xml 与 CMakeLists.txt | 通过 |
| 3 | launch 引用文件存在 | 检查 `sim.launch` 引用的 world/rviz/xacro 路径 | 通过 |
| 4 | 无个人电脑绝对路径 | 全文搜索 `C:\`、`/home/`、`/Users/` | 通过 |
| 5 | 轮关节名一致 | 比对 xacro 关节名与差速插件 `leftJoint`/`rightJoint` | 通过 |
| 6 | 轮距 / 轮径一致 | 比对 xacro 属性与插件 `wheelSeparation`/`wheelDiameter` | 通过 |
| 7 | 传感器话题 / frame_id 一致 | 比对插件参数与接口表 | 通过 |
| 8 | TF 无重复发布 | 插件 `publishWheelTF=false`，轮 TF 由 robot_state_publisher 发布 | 通过 |
| 9 | 必要文件齐全 | 按目录结构核对 | 通过 |
| 10 | Git diff 无无关改动 | `git status` / `git diff --stat` | 通过（仅本轮文件） |

执行过的 XML 解析命令（Python 标准库）：

```bash
python - <<'PY'
import xml.etree.ElementTree as ET
files = [
    "catkin_ws/src/aigc_description/package.xml",
    "catkin_ws/src/aigc_description/urdf/robot.urdf.xacro",
    "catkin_ws/src/aigc_sim/package.xml",
    "catkin_ws/src/aigc_sim/launch/sim.launch",
    "catkin_ws/src/aigc_sim/worlds/test.world",
]
for f in files:
    ET.parse(f)
    print("OK", f)
PY
```

> 注意：`robot.urdf.xacro` 包含 xacro 属性与 `${...}` 表达式，XML 语法解析通过**不等于** xacro 展开、URDF 合法或 Gazebo 运行通过。`.rviz` 为 YAML 格式，未纳入上述 XML 解析（其内容为人工核对）。
>
> **本次修复记录**：上一版把 ROS1 Melodic 插件参数误写成下划线名称（如 `command_topic`、`camera_name`、`topic_name`、`left_joint`、`wheel_separation`、`publish_odom` 等），这些并不是 Melodic 三个插件的有效参数名。`libgazebo_ros_diff_drive.so`、`libgazebo_ros_camera.so`、`libgazebo_ros_laser.so` 在 Melodic 中实际使用驼峰名称（如 `commandTopic`、`cameraName`、`topicName`、`leftJoint`、`wheelSeparation`）。本次已按任务单给出的 Melodic 参数修正，并显式配置 `odometrySource=encoder`；`always_on` 作为 sensor 的 SDF 原生参数，`update_rate` 仅保留在 sensor 下，插件内的下划线参数已删除。**未联网**，未能在线复核官方源码，按任务单明确参数执行；此前的“插件参数静态检查通过”结论不准确，予以纠正。

### B. 静态审查结果

- **模型**：使用基础几何体（box/cylinder/sphere），无外部 STL、无在线下载；单位统一为米/千克/弧度；实体均有 visual、collision、质量和惯量。
- **驱动轮**：圆柱绕 y 轴（`rpy="1.5708 0 0"`），关节轴 `0 1 0`，连续关节；左轮 `+y`、右轮 `-y`；驱动轮接地（轮心 z=0.035=半径）。
- **支撑**：前后各一个低摩擦球（`mu1/mu2=0.01`），不阻碍转向。
- **差速插件**：`libgazebo_ros_diff_drive.so`，`robotNamespace=/`、`updateRate=30`，`leftJoint=left_wheel_joint`、`rightJoint=right_wheel_joint`，`wheelSeparation=0.16`、`wheelDiameter=0.07`，`commandTopic=/cmd_vel`、`odometryTopic=/odom`、`odometryFrame=odom`、`robotBaseFrame=base_footprint`，`odometrySource=encoder`（编码器里程计）；`publishTf=true`、`publishOdomTF=true` 发布 `odom → base_footprint`；`publishWheelTF=false`（轮 TF 交给 robot_state_publisher）；`publishWheelJointState=true`。
- **相机**：`libgazebo_ros_camera.so`，640×480，sensor 下 `update_rate=15.0`（频率由 sensor 控制，插件 `updateRate=0`）；插件 `robotNamespace=/`、`cameraName=usb_cam`、`imageTopicName=image_raw`、`cameraInfoTopicName=camera_info`、`frameName=camera_optical_frame`，`always_on` 为 sensor 的 SDF 原生参数；话题 `/usb_cam/image_raw`、`/usb_cam/camera_info`；optical frame 通过固定关节 `rpy="-1.5708 0 -1.5708"` 得到 z 前、x 右、y 下。
- **激光**：`libgazebo_ros_laser.so`，360°、360 采样，sensor 下 `update_rate=10.0`（频率由 sensor 控制）；插件 `robotNamespace=/`、`topicName=/scan`、`frameName=laser_link`；扫描平面距地面约 0.20 m（墙高 0.30 m 可被扫到）。
- **场地**：地面与光照均内联；围墙内净尺寸 3.6 m × 3.6 m（内侧面 ±1.8 m），墙厚 0.10 m，墙高 0.30 m；前方 0.8 m 处有彩色测试物体；机器人出生点 (0,0) 与场地、物体不重叠。
- **启动**：`use_sim_time=true`、加载 world、xacro 生成 `robot_description`、spawn 机器人（z=0.02 防穿地）、robot_state_publisher、可选 RViz；支持 `gui`/`rviz`/`paused` 参数，默认不暂停。
- **RViz**：Fixed Frame=`odom`，显示 RobotModel、LaserScan（`/scan`）、Image（`/usb_cam/image_raw`）、TF；不创建 map 坐标系，无额外静态 TF。

### C. 待 Ubuntu 执行的验证

以下全部**待执行**，本轮不以下列项目通过作为提交前提：

- [ ] `xacro` 展开（`rosrun xacro xacro .../robot.urdf.xacro`）
- [ ] `check_urdf`（URDF 合法性）
- [ ] `catkin_make` 编译
- [ ] Gazebo 启动、机器人出生与稳定
- [ ] 话题与频率：`/clock`、`/odom`、`/scan`、`/usb_cam/image_raw`、`/usb_cam/camera_info`、`/joint_states`
- [ ] TF：`odom → base_footprint`、`base_footprint → base_link`、`base_link → laser_link`、`base_link → camera_optical_frame`
- [ ] 相机画面
- [ ] 雷达测距（扫描到墙与测试物体）
- [ ] 前进、后退、原地转向、停车
- [ ] 模型运动稳定性

---

## 第二轮：单点导航

本轮范围：静态地图生成、AMCL 定位、move_base 全局/局部规划、代价地图与避障、目标点发送脚本、启动文件与导航 RViz。

在 Windows 上完成了静态检查与地图生成验证；**导航尚未在 Ubuntu 上实际跑通**，Ubuntu 上的编译与运行验证留待后续执行。

### A. Windows 已实际执行的检查（第二轮）

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 新 XML 文件语法 | Python `xml.etree.ElementTree` 解析 package.xml 与两个 launch | 通过 |
| 2 | 新 YAML 文件语法 | Python `yaml.safe_load` 解析 6 个 config + navigation.rviz | 通过 |
| 3 | 地图生成工具运行 | `python tools/generate_test_map.py` | 通过（提取 4 墙 + 测试物体，地面跳过） |
| 4 | PGM 头与尺寸 | 解析 P5 头，核对 200×200、maxval 255、像素数据 40000 字节 | 通过 |
| 5 | 地图像素核对（无上下颠倒、障碍/空闲正确） | 读取 PGM 抽查：原点(0,0)与目标(1.2,0)空闲；测试物体(0.8,0)与四面墙占据；上/下半区空闲 | 通过 |
| 6 | 重复生成结果一致 | 两次生成 `md5sum` 比对 | 通过（一致） |
| 7 | Python 脚本语法 | `python -m py_compile` 校验 send_goal.py、generate_test_map.py | 通过 |
| 8 | launch 引用文件存在 | 解析 `$(find pkg)/path` 并核对文件存在 | 通过 |
| 9 | 无个人电脑绝对路径 | 搜索 `C:\`、`/home/`、`/Users/` | 通过 |
| 10 | 文本文件 UTF-8 / LF | 逐文件检查 BOM、CRLF、UTF-8 | 通过（生成工具已加 `newline='\n'` 修正 test.yaml） |
| 11 | 第一轮插件驼峰参数未被改回 | grep 核对 `commandTopic` 等仍为驼峰；下划线仅为 xacro 属性与 SDF `update_rate` | 通过 |
| 12 | footprint 覆盖机器人 | 比对 footprint 矩形与 xacro 车身/车轮/相机外廓 | 通过 |

执行过的关键命令（Windows，Python 3.13 标准库 + 已安装的 PyYAML）：

```bash
# 生成地图
python tools/generate_test_map.py

# 像素核对（节选：P5 头 + 关键点 free/occupied + 无上下颠倒）
python - <<'PY'
import math
raw = open("catkin_ws/src/aigc_navigation/maps/test.pgm", "rb").read()
h = raw.split(b"\n", 3); w, hh = map(int, h[1].split())
data = raw[len(h[0])+1+len(h[1])+1+len(h[2])+1:]
assert (w, hh) == (200, 200) and len(data) == 200*200
def get(x, y):
    col = int((x + 2.0) / 0.02); row = int((2.0 - y) / 0.02)
    return data[row*200 + col]
assert get(0.0, 0.0) == 254    # 原点空闲
assert get(1.2, 0.0) == 254    # 目标空闲
assert get(0.8, 0.0) == 0      # 测试物体占据
assert get(1.85, 0.0) == 0     # 前墙占据
assert get(0.0, 1.85) == 0     # 左墙（上）占据
assert get(0.0, -1.85) == 0    # 右墙（下）占据
print("pixel checks OK")
PY

# YAML 语法
python - <<'PY'
import yaml, glob
for f in glob.glob("catkin_ws/src/aigc_navigation/config/*.yaml") + ["catkin_ws/src/aigc_navigation/rviz/navigation.rviz"]:
    yaml.safe_load(open(f, encoding="utf-8"))
print("yaml OK")
PY

# 脚本语法
python -m py_compile catkin_ws/src/aigc_navigation/scripts/send_goal.py tools/generate_test_map.py
```

### B. 静态审查结果（第二轮）

- **地图生成工具**（`tools/generate_test_map.py`，Python 3 标准库，Windows 可直接运行）：解析 `test.world` 的静态模型 box 碰撞几何；模型/连杆存在非零旋转、非 box 几何或 link 级 pose 时明确报错，不静默生成错误地图；地面按名称或“不高于 z=0”跳过；栅格化用 `floor`/`ceil`，不缩小实际碰撞范围；输出 P5 PGM（free=254、occupied=0）+ YAML。
- **地图元数据**（`maps/test.yaml`）：`image: test.pgm`、分辨率 0.02、origin `[-2.0,-2.0,0.0]`、`negate: 0`、`occupied_thresh: 0.65`、`free_thresh: 0.196`（黑=占据、白=空闲，符合 map_server 约定）。
- **定位**（`amcl.yaml`）：`odom_model_type=diff`，帧 `odom`/`base_footprint`/`map`，`use_map_topic=false`（经 static_map 服务取图），粒子 500～2000，`laser_max_range=8.0` 与雷达一致，初始位姿 (0,0,0) 与 Gazebo 出生点一致。
- **move_base**（`move_base.yaml` + launch）：`base_global_planner=navfn/NavfnROS`、`base_local_planner=dwa_local_planner/DWAPlannerROS`；规划频率 1 Hz、控制频率 5 Hz；恢复行为 conservative_reset/aggressive_reset/rotate_recovery。
- **代价地图**：`costmap_common.yaml` 以 `ns=global_costmap` 与 `ns=local_costmap` 各加载一次；全局为 `map` 固定、静态层+障碍层+膨胀层、非滚动；局部为 `odom` 固定、障碍层+膨胀层、滚动窗口 3 m×3 m；`footprint`/`footprint_padding`/`transform_tolerance` 在 costmap 顶层，`observation_sources` 及 `scan` 观测源位于 `obstacle_layer` 命名空间下（最终为 `/move_base/{global,local}_costmap/obstacle_layer/observation_sources`），`inflation_radius`/`cost_scaling_factor` 位于 `inflation_layer` 下；观测源 `/scan`（marking+clearing，`max_obstacle_height=0.35`、`min_obstacle_height=0.02`）；footprint 为覆盖车身/车轮/相机的最小矩形，膨胀半径 0.30 m。
- **DWA**（`dwa_local_planner.yaml`）：使用 DWAPlannerROS 真实参数名（非其他局部规划器参数）；`max_vel_x=0.15`、`max_vel_y=0`（差速、无横向）、`min_vel_theta=0.10`（最小旋转速度绝对值，方向由 DWA 采样处理）、`min_vel_trans=0.02`、`vy_samples=1`、`xy_goal_tolerance=0.10`、`yaw_goal_tolerance=0.15`；不包含 `min_in_place_vel_theta`（非当前 DWAPlannerROS 参数）；sim 参数与评分项齐全；以上均为待 Ubuntu 调整的初值，未声称已调优。
- **send_goal.py**：Python 2.7/3 兼容（`time.monotonic` 回退到 `time.time`，`Thread.daemon` 用属性设置）；`SimpleActionClient('move_base', MoveBaseAction)`；目标在 `map` 帧；私有参数 `~x/~/y/~yaw`（默认 1.2/0/0）与 `~wait_timeout`/`~result_timeout`（单位均为**实际墙上秒**，与 `/clock` 无关）；服务连接等待与目标执行均以墙上时间超时（连接等待在守护线程中完成，只等连接、不发送目标，超时后主程序退出）；目标执行轮询本客户端 Action 状态并用普通 `time.sleep`；只有 `GoalStatus.SUCCEEDED` 算成功；清理可重复调用、各步异常互不打断；仅在确实发送且尚未结束时 `cancel_goal()`（不 `cancel_all_goals`）；停车用普通 `time.sleep` 重复发布零速度；退出码 0/1/2/3/130 分别对应成功/服务不可用/超时/失败/中断或 ROS shutdown；连接结果取 `bool(client.wait_for_server())` 的实际返回值；在服务连接等待返回后、发送目标前、目标执行返回后三处主动检查 `rospy.is_shutdown()`（ROS 的 SIGINT 通常只置位 `is_shutdown()`、不再抛 `KeyboardInterrupt`），检测到即清理并返回 130，且 shutdown 后不再发送目标；`KeyboardInterrupt` 仍覆盖连接等待与目标执行两个阶段。
- **启动**：`navigation.launch` 只启动 map_server、AMCL、move_base 与可选导航 RViz，`use_sim_time=true`；`nav_sim.launch` 包含 `sim.launch`（关闭其 RViz）+ `navigation.launch`，支持 `gui`/`rviz`/`paused` 参数；参数命名空间与插件对应（`DWAPlannerROS`、costmap 插件层、`recovery_behaviors`）。
- **RViz**（`navigation.rviz`）：Fixed Frame=`map`；显示 Map、全局/局部代价地图、RobotModel、LaserScan、TF、全局路径 `/move_base/NavfnROS/plan`、局部路径 `/move_base/DWAPlannerROS/local_plan`；工具含 2D Pose Estimate（`/initialpose`）与 2D Nav Goal（`/move_base_simple/goal`）。

### C. 待 Ubuntu 执行的验证（第二轮）

以下全部**待执行**，本轮不以下列项目通过作为提交前提：

- [ ] `catkin_make` 编译（含 `aigc_navigation`）
- [ ] `map_server` 正确加载 `test.yaml`，`/map` 内容与预期一致
- [ ] AMCL 启动、定位收敛，`map → odom` 稳定
- [ ] move_base 无参数错误；costmap_2d / Navfn / DWA / 恢复行为插件加载成功
- [ ] 全局、局部代价地图正常发布，膨胀与 footprint 生效
- [ ] 全局路径生成，局部规划沿路径行进并绕开 `test_object`
- [ ] `send_goal.py` 导航到 (1.2, 0, 0) 并返回 `SUCCEEDED`（退出码 0）
- [ ] 目标不可达时 move_base 中止，脚本非零退出并停车
- [ ] RViz 各显示与「2D Nav Goal」点击导航
- [ ] 恢复行为触发（必要时）
- [ ] 障碍层实际订阅 `/scan`（`rosnode info /move_base` 或 `rostopic info` 确认观测源生效）
- [ ] 新增激光可见障碍后，局部代价地图能够更新
- [ ] 暂停仿真（`/clock` 停止）时，`send_goal.py` 仍能按墙上时间超时退出（退出码 2）
- [ ] `send_goal.py` 运行中 Ctrl+C 后能正常取消目标并停车（退出码 130）
- [ ] 导航运行中节点被 shutdown（`rosnode kill` 或 launch 关闭）时，`send_goal.py` 主动检测 `rospy.is_shutdown()` 并返回 130（不误报为连接失败或超时）

### D. 第二轮修复记录

上一版存在四处静态问题，本次已修复（地图、多点任务、视觉等未改动）：

1. **依赖声明**：`CMakeLists.txt` 原用 `find_package(catkin REQUIRED COMPONENTS ...)` + `catkin_package(CATKIN_DEPENDS ...)`，而 `package.xml` 对这些组件只声明了 `exec_depend`，会触发 catkin 检查错误。已改为 `find_package(catkin REQUIRED)` + `catkin_package()`；`package.xml` 保留 `buildtool_depend(catkin)` 与各 `exec_depend`；不引入 C++ 编译目标。
2. **costmap 参数层级**：原把 `observation_sources` 及 `scan` 放在 costmap 顶层，但 costmap_2d 的 obstacle_layer 插件从自身命名空间读取这些参数。已改为 `obstacle_layer:` 下放 `observation_sources`/`scan`/`obstacle_range`/`raytrace_range`，`inflation_layer:` 下放 `inflation_radius`/`cost_scaling_factor`；最终参数为 `/move_base/{global,local}_costmap/obstacle_layer/observation_sources`。`navigation.launch` 不变（common 文件仍分别加载到 `global_costmap` 与 `local_costmap`）。
3. **DWA 参数**：`min_vel_theta` 改为 0.10（最小旋转速度绝对值，非右转下限，方向由采样处理）；删除非当前 DWAPlannerROS 使用的 `min_in_place_vel_theta`；`vy_samples` 改为 1（横向速度上下限仍为 0）；`min_vel_trans` 先设为 0.02。其余低速上限与评分项保留，均为待 Ubuntu 调整的初值。
4. **send_goal.py 超时/退出**：原先 `wait_for_server`/`wait_for_result` 依赖 ROS 时间、`publish_stop` 用 `rospy.sleep`，`/clock` 不更新或仿真暂停时无法按预期结束。已改为：服务连接等待与目标执行均以墙上时间为准（`time.monotonic` 回退 `time.time`）；连接等待在守护线程中完成（只等连接、不发送目标），超时后主程序可退出；目标执行轮询本客户端 Action 状态并用普通 `time.sleep`；停车用普通 `time.sleep`；清理可重复调用、各步异常互不打断；仅在确实发送且未结束时 `cancel_goal()`（不 `cancel_all_goals`）；退出码 0/1/2/3/130。

Windows 重新检查（本轮修复，均已实际执行）：

- [x] XML / YAML / Python 语法（`py_compile` 校验 send_goal.py）
- [x] CMake 与 package.xml 一致性（CMake 无 `COMPONENTS`/`CATKIN_DEPENDS`/C++ 目标；package.xml 仅 `buildtool_depend` + `exec_depend`）
- [x] costmap 插件参数层级（模拟 rosparam 加载，确认 `obstacle_layer/observation_sources`、`inflation_layer/inflation_radius` 落在正确命名空间，global/local 均覆盖）
- [x] DWA 参数名称与含义（`min_vel_theta=0.10`、无 `min_in_place_vel_theta`、`vy_samples=1`、`min_vel_trans=0.02`）
- [x] send_goal.py 分支：用简单替身（stub）验证 `wait_for_server_wall` 连接/超时、`wait_for_result_wall` 成功/超时、`publish_stop` 容忍失败发布器、`yaw_to_quaternion`（替身仅验证分支逻辑，**不代表 ROS 实际运行通过**）

> 纠正上一版 B 节结论：依赖声明未单独核对（CMake 与 package.xml 不一致）、costmap 观测源误放在顶层、DWA 误含 `min_in_place_vel_theta` 且 `min_vel_theta` 为负值、脚本超时依赖 ROS 时间。以上均已在本次修复中更正。

### E. 第二轮中断处理修复（二）

上一版 `send_goal.py` 的中断处理仍有两处问题，本次已修复（仅改脚本与验证记录，未扩展功能）：

1. **连接结果取实际返回值**：原守护线程中只要 `client.wait_for_server()` 返回就置 `connected=True`，未使用其返回值。已改为 `connected['ok'] = bool(client.wait_for_server())`，仅在服务真正连接时判真。
2. **不依赖 `KeyboardInterrupt` 判断中断**：ROS 的 SIGINT 处理通常只置位 `rospy.is_shutdown()`，不再抛出 `KeyboardInterrupt`。已在三处主动检查 shutdown：服务连接等待返回后、发送目标前、目标执行返回后；检测到即执行清理并返回 130，且 shutdown 后不再发送目标；shutdown 不再被误报为连接失败（1）或执行超时（2）。
3. **`KeyboardInterrupt` 仍保留**：覆盖连接等待与目标执行两个阶段；清理仍只取消本客户端已发送且尚未结束的目标（不 `cancel_all_goals`），停车用普通 `time.sleep`。

退出码保持 0/1/2/3/130，Python 2.7 兼容不变。

Windows 重新检查（本轮，均已实际执行；用简单替身 stub 验证分支逻辑，**不代表 ROS 实际运行通过**）：

- [x] `python -m py_compile` 语法通过
- [x] `wait_for_server()` 返回 False → 辅助函数同样返回 False
- [x] `wait_for_server()` 返回 True → 辅助函数返回 True
- [x] 连接等待期间 shutdown → 退出 130，`send_goal` 调用次数为零
- [x] 目标执行期间 shutdown → 取消目标（`cancel_goal` 至少一次）并退出 130
- [x] 服务未连接 → 退出 1，零发送
- [x] 执行超时 → 退出 2，发送一次并取消
- [x] 成功 → 退出 0，不取消

---

## 第三轮：全程航点导航与比赛主流程骨架

本轮范围：航点配置、顺序导航、任务点接口、失败重试、超时/中断处理、运行记录、launch 文件、共享模块 `navigation_client`、`send_goal.py` 重构。

起始 commit：`df50734687b6eaace91f1e63cd4a3499ad5847b7`

> 本轮只做一次本地提交；**未在 Ubuntu 实际运行，全程仿真尚未跑通**。多点巡航调用方式**参考**原 `abot_patrol_nav.py` 巡航示例（`SimpleActionClient("move_base", MoveBaseAction)` + 保存目标列表 + 顺序发送 + 读取动作结果），**未迁移整个原源码包**。

### A. Windows 已实际执行的检查（第三轮）

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 新增/改动 Python 语法 | `python -m py_compile` 校验 `send_goal.py`、`mission_manager.py`、`navigation_client.py`、`__init__.py`、`setup.py` | 通过 |
| 2 | 新增 launch XML 语法 | `xml.etree.ElementTree` 解析 mission.launch、mission_sim.launch 及既有 launch | 通过 |
| 3 | mission.yaml 与既有 YAML 语法 | `yaml.safe_load` | 通过 |
| 4 | catkin Python 安装配置 | 核对 `catkin_python_setup()`、`setup.py`（`packages=['aigc_navigation']`、`package_dir={'':'src'}`）、`catkin_install_python` 含两个脚本 | 通过 |
| 5 | 依赖声明 | package.xml 含 rospy/actionlib/move_base_msgs/geometry_msgs/tf 等 exec_depend | 通过 |
| 6 | launch 引用与命名空间 | 核对 mission.launch / mission_sim.launch 的 `$(find ...)` 路径、`<rosparam>` 加载、`log_dir` 参数 | 通过 |
| 7 | 共享模块不 init_node / sys.exit | 检查 navigation_client.py 无 `rospy.init_node` / `sys.exit` | 通过 |
| 8 | send_goal.py 仍可用 | 重构后仍 `from ... import NavigationClient`，命令接口与退出码不变 | 通过 |
| 9 | 航点与固定障碍物不重叠 | 脚本核对起点/4 航点与墙/测试物体距离（footprint 含 padding 半径约 0.212 m） | 通过（全部 CLEAR，最小约 0.34 m） |
| 10 | 无个人电脑绝对路径 | 搜索 `C:\`、`/home/`、`/Users/` | 通过 |
| 11 | 文本 UTF-8 / LF | 逐文件检查 BOM、CRLF、UTF-8 | 通过 |
| 12 | 第一轮驼峰参数未改回 | grep 核对 `commandTopic` 等仍为驼峰 | 通过 |
| 13 | Git diff 无无关改动 | `git status` / `git diff --stat` | 通过（仅本轮文件） |

### B. 替身（stub）检查（第三轮，不依赖 ROS）

用注入 fake `rospy`/`actionlib`/`move_base_msgs`/`geometry_msgs`/`tf` 的替身驱动 `mission_manager.py` 与重构后 `send_goal.py` 的 `main()`，验证分支逻辑（**不代表 ROS 实际运行通过**）。以下 58 项断言全部通过：

- [x] 四个航点全部成功：顺序发送 (1.2,0)→(1.2,1.0)→(-1.0,1.0)→(-1.0,-1.0)，每个仅发一次，退出 0，记录 COMPLETED
- [x] 任务接口只在 `task` 航点触发（不在 `finish` 触发）
- [x] 首次失败 → 重试成功，该航点尝试次数记为 2
- [x] 重试耗尽 + `failure_policy=stop` → 退出 2，状态 FAILED
- [x] `failure_policy=skip` → 继续后续航点，退出 3，状态 PARTIAL，任务点记 skipped
- [x] `transit` 失败 → 停止、非 COMPLETED；`finish` 失败 → 停止、非 COMPLETED
- [x] 执行超时 → 取消 → 确认 → 重试（取消一次、共发 5 次目标）
- [x] 取消未确认 → 不再发送后续目标，退出 2
- [x] shutdown → 不再发送后续目标，退出 130
- [x] 服务未连接 / 时钟未启动 → 就绪超时，退出 1，零发送
- [x] 成功后不取消已完成目标（cancel 次数为 0）
- [x] 同进程重跑两次：状态重置（两次各发 4 个目标，`run_id` 不同）
- [x] 配置校验：重名/空列表/finish 非最后/负超时/max_retries 负数·小数·布尔/failure_policy 非法/NaN·Inf 坐标均报错
- [x] 重构后 send_goal.py：服务不可用→1、成功→0、超时→2（且取消）、shutdown→130、失败发布器被容忍

### C. 静态审查结果（第三轮）

- **共享模块**（`navigation_client.py`）：只做 Action 客户端与停车，不调用 `init_node`/`sys.exit`；`TERMINAL_STATES` 与 `DEFINITIVE_TERMINAL_STATES`（后者排除 LOST，因 LOST 不代表取消被确认）；所有等待用墙上时间（`time.monotonic` 回退 `time.time`）；`cancel_goal` 只取消本客户端未结束目标（不 `cancel_all_goals`）；`cancel_and_stop` 可重复调用。
- **主流程**（`mission_manager.py`）：init → 等待就绪（服务 + TF + 时钟，墙上时间有界、可中断）→ 顺序导航 → 任务接口 → finish → 保存记录退出；单一持久 Action 客户端；`max_retries=1` 表示首次失败后再试 1 次（共 2 次），不会无限重试；`transit` 失败停止、`finish` 失败不算完成、`skip` 最终 PARTIAL、LOST/取消未确认停止；超时先取消再确认，确认才重试，未确认停止。
- **任务接口**：当前为占位（打印提示 + 停留 `dwell_time`），识别/对准/击打未接入。
- **运行记录**：每次启动新 `run_id`，写 `~/.ros/aigc/`（可 `log_dir` 覆盖），字段含 run_id/起止时间/状态/退出码/原因/耗时/逐航点记录；成功/失败/中断均保存，不覆盖；保存失败不改写退出码。
- **launch**：`mission.launch` 仅 mission_manager + mission.yaml + log_dir；`mission_sim.launch` 包含 nav_sim.launch + mission_manager，支持 `gui`/`rviz`/`paused`/`mission_file`/`log_dir`。
- **配置校验**：坐标有限、名称唯一、列表非空、finish 最后、超时为正、max_retries 非负整数、failure_policy 限 stop/skip。
- **航点坐标**：四点均在 3.6 m × 3.6 m 内且不落入墙/测试物体；机器人 footprint（含 padding 半径约 0.212 m）与障碍最小距离约 0.34 m。

### D. 待 Ubuntu 执行的验证（第三轮）

以下全部**待执行**，本轮不以下列项目通过作为提交前提；**未在 Ubuntu 实际运行，全程仿真尚未跑通**：

- [ ] `catkin_make` 编译，含 `catkin_python_setup()`/`setup.py` 安装共享模块 `aigc_navigation`
- [ ] `mission_manager` 节点启动、`mission.yaml` 正确加载到私有命名空间
- [ ] 依次导航四个航点并到达 `finish`，退出码 0，记录 COMPLETED
- [ ] 到达任务点时触发占位任务接口（日志可见）
- [ ] 航点失败重试、`max_retries` 耗尽后按 `failure_policy` 停止/跳过
- [ ] 执行超时 → 取消 → 确认 → 重试；取消未确认 → 停止
- [ ] Ctrl+C / shutdown → 退出 130 且保存运行记录
- [ ] 运行记录写入 `~/.ros/aigc/`，字段完整，重跑不覆盖
- [ ] `send_goal.py` 重构后仍可单独导航（行为不变）
- [ ] 暂停仿真（`/clock` 停止）时任务管理器仍按墙上时间超时/退出

### E. 第三轮修复记录（就绪检查与目标生命周期）

上一版存在若干问题，本次已修复（接口、地图、场地、导航参数不变）：

1. **就绪检查未真正受墙上时间限制**：上一版 TF 就绪用 `tf_listener.waitForTransform(..., rospy.Duration(0.2))`，其超时依赖 ROS 时间；当 `use_sim_time=true` 且 `/clock` 暂停时该调用可能长时间阻塞，无法按预期退出。已改为非阻塞 `canTransform(frame, 'base_footprint', rospy.Time(0))` 轮询 + 普通 `time.sleep`，每轮检查墙上 deadline 与 `is_shutdown()`；服务连接复用 `NavigationClient.wait_for_server(timeout)`，`server_timeout` 真正生效（连接等待 ≤ `server_timeout`，就绪总时长 ≤ `ready_timeout`，后续 TF/时钟等待只用剩余时间）；就绪失败/中断时零发送。**纠正上一版“就绪检查墙上时间有界”的不准确结论**：上一版该结论只覆盖服务连接等部分，TF 等待仍依赖 ROS 时间，本次才真正做到全流程与 `/clock` 无关。
2. **目标所有权**：上一版 `cancel_goal()` 发出取消请求后即置 `_goal_sent=False`，`wait_for_result()` 遇到 LOST 也置 False，会把“取消未确认/未知”误判为“已结束”。已改为所有权仅在**确定的终态**（SUCCEEDED/ABORTED/REJECTED/PREEMPTED/RECALLED）释放；LOST 保留所有权；允许取消自己处于 LOST 的目标；取消超时/异常/shutdown 保持未确认状态（清理可重试）；不 `cancel_all_goals`；旧目标未确认时不发送新目标；清理有限次、可重复、如实记录。
3. **取消确认过程中的中断**：上一版 `wait_for_cancel()` 返回 False 后直接按“取消未确认”处理。已改为先检查 shutdown：shutdown → 清理并退出 130（记录 STOPPED，含当前航点/尝试次数/中断原因）；普通未确认 → 退出 2；已确认 → 允许重试；其它等待与停留之后同样检查 shutdown。
4. **kind 校验统一**：上一版 `validate_config` 用 `wp.get('kind', 'task')`，运行期用 `wp['kind']`，缺少 `kind` 会通过校验却在运行期崩溃。已改为 `kind` 必填且仅限 `task`/`transit`/`finish`，缺失给出明确配置错误并退出 1，校验失败不发送任何目标。

Windows 重新检查（本轮修复，均已实际执行；用简单替身 stub 验证分支逻辑，**不代表 ROS 实际运行通过**）：

- [x] `python -m py_compile` 校验 5 个 Python 文件
- [x] 替身回归 **74 项断言全部通过**（含既有分支与新场景），其中新增：
  - [x] TF 永不存在 + `/clock` 冻结 → 按墙上时间退出 1、零发送，且 `waitForTransform` 从未被调用（`canTransform` 被轮询）
  - [x] `server_timeout` < `ready_timeout` → 连接等待被 `server_timeout` 限定，未拖到 `ready_timeout`
  - [x] 取消已发出但目标仍 ACTIVE → 所有权保留、清理重试取消、不再发送新目标
  - [x] LOST → 停止运行（退出 2、记录 lost），LOST 不被当作取消确认、所有权保留
  - [x] 取消确认期间 shutdown → 退出 130、状态 STOPPED、记录 interrupted 且不再发后续目标
  - [x] 缺少 kind → 配置校验失败、退出 1、零发送
  - [x] 既有全成功/重试/停止/跳过/transit 失败/finish 失败/超时取消确认，及 send_goal.py 各分支，全部保持通过
- [x] `git diff --check` 无空白错误；UTF-8 / LF 逐文件检查通过
