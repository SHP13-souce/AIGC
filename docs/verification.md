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

---

## 第四轮：AR 测试靶识别、对准与模拟击打

本轮范围：官方 `ar_track_alvar` 测试靶检测、按方位误差原地对准、`/shoot` 接口模拟击打、串联现有比赛主流程；新增 `task_executor.py`、`sim_shooter.py`、`config/task.yaml`、`launch/ar_detector.launch`、`launch/task_mission_sim.launch`、三个测试靶 DAE/PNG 资源，并更新 `mission_manager.py`、`mission.yaml`、`test.world`、地图、package.xml、CMakeLists.txt。

起始（基线）commit：`6a9c0414ac990854591d0f77a94b173ef87f5847`

> 本轮只做一次本地提交；**未在 Ubuntu 实际运行，全程仿真尚未跑通**。识别对象是 **9×9 AR 测试靶（非正式比赛靶）**；击打为进程内**模拟**（`backend=simulated`、`hit=null`），不声称“实测命中”。

### A. Windows 已实际执行的检查（第四轮）

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 全部 Python 语法 | `python -m py_compile`（mission_manager.py、send_goal.py、navigation_client.py、task_executor.py、sim_shooter.py、__init__.py、setup.py、两个 tools） | 通过 |
| 2 | 全部 XML 语法 | `xml.etree.ElementTree` 解析 package.xml、8 个 launch、test.world、3 个 DAE | 通过 |
| 3 | 全部 YAML 语法 | `yaml.safe_load`（8 个 config 含新增 task.yaml） | 通过 |
| 4 | PNG 尺寸/内容 | 解析 PNG IHDR/IDAT，核对 180×180、灰度（color type 0），81 格逐一与生成器 PATTERNS 一致 | 通过 |
| 5 | DAE 纹理引用 | 解析 `<init_from>`，相对路径解析后文件存在；无 CRLF | 通过 |
| 6 | 地图含测试靶障碍 | 解析 PGM：200×200、maxval 255、像素 40000；占据像素 4160（墙+物体+三靶板） | 通过 |
| 7 | 依赖 / launch 一致性 | 核对 package.xml 新增 exec_depend、ar_detector.launch 参数、task_mission_sim.launch 引用与 `gazebo_media_path` 导出 | 通过 |
| 8 | 无个人电脑绝对路径 | 搜索 `C:\`、`/home/`、`/Users/` | 通过 |
| 9 | UTF-8 / LF | 逐文件检查 BOM、CRLF、UTF-8 | 通过 |
| 10 | 第一轮驼峰参数未改回 | grep 核对 `commandTopic` 等仍为驼峰 | 通过 |
| 11 | Git diff 无空白错误 | `git diff --check` | 通过 |

### B. 替身（stub）检查（第四轮，不依赖 ROS）

用注入 fake `rospy`/消息类型/`actionlib`/`tf` 的替身驱动 `task_executor.py`、`sim_shooter.py`、`mission_manager.py` 与 `navigation_client.py`，验证分支逻辑（**不代表 ROS 实际运行通过**）。共 **79 项断言全部通过**，覆盖：

- [x] 检测过滤：无消息 / 错误 ID / 错误坐标系 / 图像时间戳过旧 / `z==0` / 坐标非有限 / 同 id 多个 marker / 墙上接收过旧 → 均不判为有效目标
- [x] 转向符号：目标偏右（`x>0`）→ `error>0` → `angular.z<0`（原地右转），且全程 `linear.x/y=0`
- [x] 无目标全流程 → 超时、不击打、不发布 `shoot`
- [x] 对准成功 → 成功、`aligned=true`、恰好一次模拟击打、`hit=null`、任务结束发布零速度
- [x] 同一图像时间戳不重复累计稳定次数（冻结时间戳永远到不了 `stable_frames`）
- [x] 每点每轮最多一次击打（`allow_shot=False` → 不重复击打，理由为 already shot）
- [x] 模拟击打失败 → 任务记为 failed（不记为成功），反馈 `success=false`、`reason` 记录
- [x] 未停车稳定（`/odom` 运动/过旧）→ 不击打、failed、reason=did not settle
- [x] 中断 → interrupted、停止转向（末条指令零速度）、不击打
- [x] 模拟脉冲：`shoot` 发布、`cleanup` 幂等结束脉冲（恰好一次 `stopshoot`）
- [x] 下一任务不复用上一任务缓存（`execute` 开头清空 marker/odom 缓存）
- [x] 导航阶段任务模块零速度发布（构造 TaskExecutor 不发布任何速度）
- [x] `_resolve_target_id`：默认映射 task_1/2/3→0/1/2、显式 `target_id` 优先、未知名→None
- [x] 配置校验：`execute_tasks=true` 时任务点必须能解析出非负整数 `target_id`；`false` 时不校验
- [x] `_run_task_waypoint`：成功继续（记录 navigation_result/target_id/task）、失败+stop→退出 2、失败+skip→PARTIAL、中断→退出 130（非超时）、同航点第二次 `allow_shot=False`
- [x] `execute_tasks=false` 原纯导航流程仍退出 0、记录 navigation_result、不写 task 详情、不调用任务执行器
- [x] 导航客户端回归：目标所有权仅在确定终态释放、LOST 保留所有权、旧目标未确认不发送新目标、取消仅在持有目标时发出、取消确认以确定终态为准、LOST 不算取消确认

### C. 静态审查结果（第四轮）

- **检测**（`ar_detector.launch`）：官方 `ar_track_alvar` 的 `individualMarkersNoKinect`；`marker_size=9.0` cm、`marker_resolution=5`、`marker_margin=2`、`output_frame=camera_optical_frame`；重映射 `camera_image`/`camera_info` 到 `/usb_cam/*`；不复制源码、不重实现、不改为 OpenCV ArUco。
- **任务执行**（`task_executor.py`）：回调只更新数据（marker/odom 及各自墙上时间戳），控制循环只读数据、不在回调里转向/击打；按 `target_id` 过滤、同 id 多个不盲取；读 marker 自身 `header.stamp`/`header.frame_id`（**不读**容器 header 或 `pose.header`）；坐标有限且 `z>0`；双重新鲜度（图像时间戳年龄 ≤ 0.3 s + 墙上接收年龄 ≤ 0.5 s，`/clock` 暂停使检测失效）；`error=atan2(x,z)`、`angular.z=clamp(-kp·error,±max)`、仅原地；同一图像时间戳不重复计数、丢失清零；未停车稳定不击打；结束时发布有限次零速度；lazy import `TaskExecutor`，纯导航入口不 import `ar_track_alvar_msgs`。
- **模拟击打**（`sim_shooter.py`）：进程内，`/shoot`（`std_msgs/String`）发 `shoot`→`stopshoot`，可中断、`cleanup` 幂等结束脉冲；每次调用新建反馈，不复用上次结果；`hit=null`、`backend=simulated`；结果由 `shooter.success` 配置。
- **主流程集成**（`mission_manager.py`）：`execute_tasks` 默认 `false`（原流程不变）；`true` 时在 `task` 航点导航成功后执行任务，成功继续、失败按 `failure_policy` 停止/跳过、中断→130（非超时）、跳过→PARTIAL、finish 失败不算完成；记录区分 `navigation_result`/`task`/`final result`，新增 `mission_mode`/`target_id`/对准结果/击打反馈/失败原因。
- **测试靶资源**：3 块 0.20×0.20×0.02 m 白板 + 0.09 m 黑色图案，中心高 0.15 m；模型级零旋转（碰撞盒分别为 0.20×0.02×0.20 或 0.02×0.20×0.20），marker 视觉用 DAE（0.09 m 平面、UV 映射）+ `gazebo_media_path` 的 `model://` URI；已纳入静态地图。
- **启动/依赖**：`task_mission_sim.launch` 一次启动（nav_sim + ar_detector + mission_manager，`execute_tasks=true`），支持 `gui`/`rviz`/`paused`/`mission_file`/`task_file`/`log_dir`，不重复启动 Gazebo/导航/RViz；package.xml 新增 `ar_track_alvar`/`ar_track_alvar_msgs`/`std_msgs`/`nav_msgs`。

### D. 待 Ubuntu 执行的验证（第四轮）

以下全部**待执行**，本轮不以下列项目通过作为提交前提；**未在 Ubuntu 实际运行，全程仿真尚未跑通**：

- [ ] `catkin_make` 编译（含新增 Python 模块与 `gazebo_media_path` 导出、DAE/纹理安装路径）
- [ ] 三块测试靶在 Gazebo 中正确显示，纹理无镜像/倒置、方向正确（图案非左右对称，朝向影响识别）
- [ ] `ar_track_alvar` 启动、识别到 ID0/1/2，`/ar_pose_marker` 各 marker 自身 header（stamp/frame_id）正确
- [ ] 检测双重新鲜度、同帧不重复计数、目标丢失清零、转向符号在实车上成立
- [ ] 对准稳定计数与模拟击打反馈（`shoot`/`stopshoot` 时序）、每点最多一次击打、模拟失败不记为成功
- [ ] `execute_tasks=true` 三点任务 + `finish` 全流程：退出码 0、记录 COMPLETED，且逐航点记录含 navigation_result/target_id/task/击打反馈
- [ ] `execute_tasks=false` 时原纯导航流程不回归（第三轮各分支）
- [ ] 暂停仿真（`/clock` 停止）时任务/就绪按墙上时间超时或退出；中断退出 130 并结束模拟脉冲

> **第四轮结论纠正（见第五轮）**：① 上一版认为 `gazebo_media_path` 可解析 `model://` URI——错误，`model://` 由 `gazebo_model_path`（GAZEBO_MODEL_PATH）解析，媒体路径只服务 DAE 相对纹理；② 上一版未做到「击打前统一最终门槛」（停车/搜索/对准/稳定/额度/超时/中断在触发瞬间重新复核）与「`task_timeout` 覆盖整项任务（含脉冲/反馈）」，这两点在第五轮才真正实现。

---

## 第五轮：修复第四轮任务代码（击发门槛与执行期限）

本轮**只修复第四轮代码，不新增比赛功能**。保留模块、AR 图案、场地、导航接口与已通过逻辑不变。

起始（基线）commit：`81d108d1b0e32a33e85f4b3adef74a7f616d4b9a`（第四轮成果）

> 本轮只做一次本地提交；**未在 Ubuntu 实际运行，全程仿真尚未跑通**。仍然不声称 Gazebo 已正确显示或完整流程已跑通。

### A. 本轮修复内容

1. **Gazebo 模型搜索路径（资源搜索路径结论纠正）**：`test.world` 使用 `model://aigc_sim/meshes/marker_N.dae`。上一版只导出了 `gazebo_media_path`，但 `model://` URI 由 **`gazebo_model_path`（GAZEBO_MODEL_PATH）** 解析，与媒体路径（GAZEBO_RESOURCE_PATH，供 DAE 相对纹理 `../materials/textures/*.png`）不是一回事。已在 `package.xml` `<export>` 增加 `<gazebo_ros gazebo_model_path="${prefix}/.." />`：源空间 `${prefix}=.../src/aigc_sim`，安装空间 `${prefix}=.../share/aigc_sim`，两种布局下 `${prefix}/..` 都是「包含 `aigc_sim` 目录的目录」，加上 `aigc_sim/meshes/...` 正好落在已安装的网格文件上；`gazebo_media_path="${prefix}"` 与 meshes/materials 安装规则保留不变。**仅核对目录结构与 DAE/XML 语法解析，不代表 Gazebo 已正确显示。**

2. **检测新鲜度与稳定计数**：`TaskExecutor` 记录任务开始时的 ROS 时间，任务开始时清空缓存/计数；有效检测要求 marker 自身 `header.stamp > 0`、图像采集晚于任务开始、图像年龄与接收年龄都在限内、ID/坐标系/坐标有限且 `z>0`。稳定计数**只累加严格递增的新图像时间戳**：同一帧重复读取不计数也不误判丢失、回退帧不计数（A,B,A 只算 2 不是 3）；真实丢失/过期/越限才清零。里程计同样校验 `odom.header.stamp` 有效性 + 年龄 + 接收年龄（**新鲜收到的、但带旧 header 的里程计不能证明已停车**）。

3. **击打前最终门槛（击打门槛结论纠正）**：击打前重新复核——任务有效、未 shutdown、未超时、目标有效/新鲜/对准、里程计新鲜且已停、稳定帧条件成立、额度未消耗。停车判定并入最终的连续帧判定。复核失败则在剩余时间内重新搜索/对准，或按 failed/timeout 返回，**绝不在门槛未过时击打**。每点每轮至多一次击打，额度在真正触发**之前**预留（先 `_fired.add(waypoint)` 再 `shoot`，异常不会重复击打）；`allow_shot=False` 不再无条件返回成功，仅当确有可复用的先前成功结果时才记成功。

4. **整项任务统一期限与中断（整项期限结论纠正）**：`task_timeout` 覆盖停车 + 搜索 + 对准 + 再次稳定 + 脉冲 + 反馈，各阶段与 `SimShooter` 传入**同一个绝对墙上 deadline**；每阶段开始与等待后都检查 shutdown 与 deadline；`_stop_and_settle` 超时/中断后**不再**回退到「最后一份里程计碰巧稳定」当作成功。`SimShooter`：启动时 shutdown/超时则不发布 `shoot`；脉冲与反馈等待都受 deadline 约束；`try/finally` 结束脉冲；中断→`interrupted`、超时→`timeout`（两者都不使用 `config success=true`）；反馈 `backend=simulated`、`hit=null`；清理有限次，超时后不再报业务成功。`TaskExecutor.execute` 用 `try/finally`；任意阶段中断→`interrupted`；`MissionRunner` 退出 130 并记 STOPPED。

5. **任务参数校验（仅 `execute_tasks=true`、在任何转向/击打之前）**：`execute_tasks` 与 `shooter.success` 为布尔；`control_rate`/`task_timeout`/检测年龄为正有限数；`stable_frames`/`stop_publish_count` 为正整数（拒绝布尔，先判 `isinstance(v, bool)`）；速度/角度/稳定阈值合理有限；`search_rate ≤ max_angular_rate`；`pulse_duration` 为正；`delay` 非负有限；话题名/坐标系为非空字符串。非法→明确错误、退出 1，不产生任何转向/击打。Python 2.7 兼容（用 `math.isnan`/`math.isinf`，无 `math.isfinite`）。

### B. Windows 已实际执行的检查（第五轮）

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 全部 Python 语法 | `python -m py_compile`（mission_manager.py、task_executor.py、sim_shooter.py、navigation_client.py、send_goal.py、tools/regression_test.py 等） | 通过 |
| 2 | package.xml / launch / world / DAE XML 语法 | `xml.etree.ElementTree` 解析 | 通过 |
| 3 | `gazebo_model_path="${prefix}/.."` 源/安装布局解析 | 核对 `${prefix}` 取值 + `aigc_sim/meshes/marker_*.dae` 存在（源空间），安装规则 `share/aigc_sim/meshes/` | 通过（仅目录结构，非 Gazebo 显示） |
| 4 | 第一轮驼峰参数未改回 | grep 核对 `commandTopic` 等仍为驼峰 | 通过 |
| 5 | 无个人电脑绝对路径 | 搜索 `C:\`、`/home/`、`/Users/` | 通过 |
| 6 | UTF-8 / LF、`git diff --check` | 逐文件 + 空白检查 | 通过 |
| 7 | 集中回归（替身 + 可控时钟） | `python tools/regression_test.py`，**65 项断言全部通过** | 通过 |

### C. 集中回归（`tools/regression_test.py`，不依赖 ROS）

用注入 fake `rospy`/消息模块（含 `.msg` 子模块）+ 可控时钟（`now`/`rostime`/`shutdown`）+ 替换 `_wall_clock`/`time.sleep`/`is_shutdown` 的替身，驱动 `task_executor.py`/`sim_shooter.py`/`mission_manager.py`，共 **12 组 65 项断言全部通过**（**不代表 ROS 实际运行通过**；正向任务测试通过真实检测/里程计回调喂入数据，运行真实过滤、计数与最终击打判断，未把关键判断替换为 True）：

- [x] 图案/地图一致（PATTERNS 与 PNG/PGM 数据）
- [x] 零时间戳 / 任务开始前图像 / 回退时间戳 → 拒绝
- [x] 同一帧重复读取不重复计数（冻结时间戳到不了 `stable_frames`）
- [x] A,B,A 只算 2 不是 3（回退不计数）
- [x] 最终停车阶段目标丢失/漂移 → 不击打
- [x] 新鲜收到的、但带旧 header 的里程计不能证明已停车
- [x] `task_timeout` 短于脉冲/反馈 → 不成功
- [x] 各阶段 shutdown → `interrupted`、退出 130、不再击打
- [x] 已开始的脉冲在中断/超时/异常时结束（`stopshoot`）
- [x] 先前模拟失败 → 下次调用不自动成功（`allow_shot=False` 且无先前成功 → failed）
- [x] 错误 ID / 重复 ID / 过期 / 非有限坐标 → 仍拒绝
- [x] 正常任务恰好击打一次、模拟成功 `hit=null`；纯导航 / 重试 / 跳过 / 取消确认 / LOST / 中断回归通过

### D. 待 Ubuntu 执行的验证（第五轮）

以下全部**待执行**，本轮不以下列项目通过作为提交前提；**未在 Ubuntu 实际运行，全程仿真尚未跑通**：

- [ ] `catkin_make` 编译后，`model://aigc_sim/meshes/...` 在**源空间与安装空间**都能被 `gazebo_model_path` 解析（不声称 Gazebo 已正确显示）
- [ ] `ar_track_alvar` 各 marker 自身 `header.stamp` 为正且随图像刷新（检测新鲜度依赖）
- [ ] 检测/里程计双重新鲜度、严格递增计数、目标丢失清零在实车上成立
- [ ] 击打前最终门槛：目标漂移/未停/超时/中断时不击打；每点至多一次击打；模拟失败不记为成功
- [ ] `task_timeout` 覆盖整项任务（含模拟脉冲/反馈）；各阶段 shutdown→130 并结束脉冲；非法任务配置→退出 1 且无转向/击打
- [ ] `execute_tasks=true` 三点任务 + `finish` 全流程退出码 0、记录 COMPLETED；`execute_tasks=false` 原纯导航流程不回归

---

## 第六轮：修复稳定帧重置、发布异常与清理期中断

本轮**只修复第五轮（第四轮任务）代码，不新增比赛功能**。保留模块、AR 图案、场地、导航接口与已通过逻辑不变。

起始（基线）commit：`3a51e441a8b84fe014b0ac48afbf7c33347a8450`

> 本轮只做一次本地提交；**未在 Ubuntu 实际运行，全程仿真尚未跑通**。仍然不声称 Gazebo 已正确显示或完整流程已跑通。

### A. 本轮修复内容（第六轮）

1. **连续稳定帧判定**（`task_executor.py` `_search_align`）：上一版在「已对准但里程计不稳」时保留 `stable` 计数，导致「停稳→移动→停稳→移动→停稳」仍会累计到 `stable_frames` 并触发射击。已改为：里程计超速、失效或过期（`_odom_stable()` 为假）时清零 `stable`，恢复后必须重新累计连续 `stable_frames` 个**新图像帧**（时间戳严格递增）均对准且停稳才击发。保留同帧不重复计数、倒退帧不算新帧的规则（`last_counted` 不清零）。

2. **射击发布异常**（`sim_shooter.py`）：上一版 `_publish` 吞掉所有异常，`Publisher.publish` 恒抛异常时 `shoot` 仍返回 `success=true`。已改为：`_publish` 返回错误信息；`shoot` 起始命令发布失败→`failed`（未触发、不报成功）；`stopshoot` 发布失败→`failed`；`stop_pulse` 无论成败都清 `_pulse_active`（清理有限、不无限重试）；`try/finally` 仍保证异常路径尝试结束脉冲。

3. **清理期间中断**（`task_executor.py` `execute`）：上一版 `execute` 的 `finally` 停车期间若发生 shutdown，会被先前「did not settle」等普通结果掩盖，任务管理器最终返回 `(2, FAILED, did not settle)`。已改为：`finally` 清理后检查 `is_shutdown()`，若在清理期间发生 shutdown，统一把结果覆盖为 `interrupted`，优先于普通失败/超时/skip；任务管理器据此退出 130 / STOPPED。

### B. Windows 已实际执行的检查（第六轮）

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 全部 Python 语法 | `python -m py_compile`（task_executor.py、sim_shooter.py、mission_manager.py、navigation_client.py、regression_test.py） | 通过 |
| 2 | 集中回归（替身 + 可控时钟） | `python tools/regression_test.py`，**78 项断言全部通过** | 通过 |
| 3 | 第一轮驼峰参数未改回 | grep 核对 `commandTopic` 等仍为驼峰 | 通过 |
| 4 | UTF-8 / LF、`git diff --check` | 逐文件 + 空白检查 | 通过 |

### C. 新增复现用例（`tools/regression_test.py`，不依赖 ROS，**不代表 ROS 实际运行通过**）

正向用例继续通过真实检测/里程计回调喂入数据，运行真实过滤、计数与最终击打判断，关键判定未替换为固定 True：

- [x] **交替停稳/移动不能累计成功**：`(停稳, 移动, 停稳, 移动, 停稳)` 脚本化喂入 → 返回 `timeout`（不击发）
- [x] **随后连续合格新帧才允许射击**：一次移动清零后，连续 3 个递增新图像帧均对准且停稳 → 返回 `success`
- [x] **shoot 发布异常**：`Publisher.publish` 恒抛异常 → `failed`、reason 含 `shoot`、无命令上达、不启动脉冲
- [x] **stopshoot 发布异常**：起始成功、停止抛异常 → `failed`、reason 含 `stopshoot`、脉冲标志清除、`shoot` 命令恰好一次
- [x] **清理期 shutdown（任务层）**：初始停车失败后 `finally` 停车期间 shutdown → 任务返回 `interrupted`、无 `shoot`
- [x] **清理期 shutdown（任务管理器层）**：同一场景经 `_run_task_waypoint` → 退出 130 / STOPPED（优先于 `failure_policy=skip`）

### D. 待 Ubuntu 执行的验证（第六轮）

以下全部**待执行**，本轮不以下列项目通过作为提交前提；**未在 Ubuntu 实际运行，全程仿真尚未跑通**：

- [ ] 稳定帧在里程计超速/失效/过期时清零、恢复后需连续合格新帧才击发（实车行为）
- [ ] 真实 `Publisher.publish` 异常路径（开始/停止命令失败均不报成功、脉冲被有限清理）
- [ ] 任务清理期间 shutdown → 任务 `interrupted`、任务管理器退出 130 / STOPPED（优先于 skip）
- [ ] `execute_tasks=true` 三点任务 + `finish` 全流程退出码 0；`execute_tasks=false` 原纯导航流程不回归

---

## 第七轮：修复清理中断时射击记录丢失

本轮**只修复第六轮（提交 `63920c5`）的清理中断记录问题，不新增比赛功能**。

起始（基线）commit：`63920c5bf6df1c23705690054151054f78bdd0ae`

> 本轮只做一次本地提交；**未在 Ubuntu 实际运行，全程仿真尚未跑通**。

### A. 本轮修复内容（第七轮）

上一版 `task_executor.py` 的 `execute` 在 `finally` 后把结果覆盖为 `TASK_INTERRUPTED` 时，`_result` 的第三参数误写为 `None`，导致已完成的射击反馈被抹掉：任务返回 `interrupted` 但 `shot=None`，任务管理器也无法登记射击额度。

已改为把该处第三参数改为 `result.get('shot')`：覆盖状态为 `interrupted` 时保留 `result` 中已有的 `shot`（以及已有的 `aligned`），不抹掉已发生的射击、不改写其原始反馈。

### B. Windows 已实际执行的检查（第七轮）

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | Python 语法 | `python -m py_compile`（task_executor.py、regression_test.py） | 通过 |
| 2 | 集中回归 | `python tools/regression_test.py`，**84 项断言全部通过** | 通过 |
| 3 | `git diff --check`、UTF-8 / LF | 空白检查 + 逐文件 | 通过 |

### C. 新增复现用例（第七轮，不依赖 ROS，**不代表 ROS 实际运行通过**）

正向用例继续通过真实检测/里程计回调喂入数据，运行真实过滤、计数与最终击打判断，关键判定未替换为固定 True：

- [x] **射击成功后清理中断**：真实回调驱动完成 `shoot`/`stopshoot` 后，`finally` 零速度停车期间 shutdown → 任务返回 `interrupted`，且 `shot` 保留、`shot.status == success`
- [x] **模拟射击失败后清理中断**：`shooter.success=false` 下同样场景 → 任务返回 `interrupted`，且 `shot` 保留、`shot.status == failed`（未被改写为成功）
- [x] **任务管理器层**：带 `shot` 的 `interrupted` 结果经 `_run_task_waypoint` → 退出 130 / STOPPED，且该航点登记到 `shot_waypoints`

### D. 待 Ubuntu 执行的验证（第七轮）

以下全部**待执行**，本轮不以下列项目通过作为提交前提；**未在 Ubuntu 实际运行，全程仿真尚未跑通**：

- [ ] 实车上「射击完成后清理期 shutdown」时，任务记录保留射击反馈、任务管理器退出 130 / STOPPED 并登记射击额度


## Ubuntu 联调实测记录：2026-10-22

依据用户提供的 Ubuntu 虚拟机运行结果：
- 里程计约 30 Hz、激光约 10 Hz、相机约 15 Hz，运动测试通过。
- 单点导航返回 SUCCEEDED，退出码 0。
- encoder 里程计模式下，多航点导航在 task_3 失败，原因仍待排查。
- 切换 world 里程计后，四个航点全部成功，COMPLETED / 0。
- 修正三个靶标材质的 emission 和 ambient 后，完整任务通过：
  三个任务点均 aligned=True、shot_success=True，终点成功。
- 本次成功基准使用 Gazebo world 真值里程计；射击为模拟反馈。
  轮速里程计链路、正式比赛靶与实车表现仍待验证。
