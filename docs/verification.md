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
- **代价地图**：`costmap_common.yaml` 以 `ns=global_costmap` 与 `ns=local_costmap` 各加载一次；全局为 `map` 固定、静态层+障碍层+膨胀层、非滚动；局部为 `odom` 固定、障碍层+膨胀层、滚动窗口 3 m×3 m；观测源 `/scan`（marking+clearing，`max_obstacle_height=0.35`、`min_obstacle_height=0.02`）；footprint 为覆盖车身/车轮/相机的最小矩形，膨胀半径 0.30 m。
- **DWA**（`dwa_local_planner.yaml`）：使用 DWAPlannerROS 真实参数名（非其他局部规划器参数）；`max_vel_x=0.15`、`max_vel_y=0`（差速、无横向）、`xy_goal_tolerance=0.10`、`yaw_goal_tolerance=0.15`，sim 参数与评分项齐全。
- **send_goal.py**：Python 2.7/3 兼容；`SimpleActionClient('move_base', MoveBaseAction)`；目标在 `map` 帧；私有参数 `~x/~/y/~yaw`（默认 1.2/0/0）与 `~wait_timeout`/`~result_timeout`；只有 `GoalStatus.SUCCEEDED` 算成功；超时/退出时 `cancel_goal()`（仅取消自己的目标，不 `cancel_all_goals`）并发布零速度停车；Ctrl+C 清理并退出码 130；失败非零退出。
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
