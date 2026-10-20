# AIGC 无人车仿真底座

无人车挑战赛的仿真代码，目前包含三轮：

- **第一轮**：最小仿真底座——可运动的差速机器人、测试场地、相机与激光雷达。
- **第二轮**：单点自主导航——基于已有 ROS1 导航包的定位（AMCL）、全局路径规划（navfn）、局部规划（DWA）与避障，支持导航到单个目标点。
- **第三轮**：全程航点导航与比赛主流程骨架——按配置顺序导航多个航点，到达任务点调用（占位）任务接口，含失败重试、超时/中断处理与 JSON 运行记录。

> 航点导航与比赛主流程骨架已实现，但**视觉识别、对准、击打、大模型服务、正式比赛场地与真实机器人模型等暂未实现**。

---

## 1. 当前实现内容

### 第一轮：仿真底座

- 简化差速机器人模型（基础几何体，无外部 STL、无在线模型下载）
  - 矩形车身、左右驱动轮、前后低摩擦支撑
  - 支持前进、后退、原地转向
- 相机（640×480 @ 15 Hz，朝正前方，输出 `/usb_cam/image_raw`、`/usb_cam/camera_info`）
- 二维激光雷达（约 360°、360 采样点 @ 10 Hz，输出 `/scan`）
- 测试场地（地面、光照、3.6 m × 3.6 m 围墙、前方彩色测试物体）
- 启动文件 `sim.launch`（Gazebo + robot_state_publisher + 可选 RViz）
- RViz 配置（RobotModel / LaserScan / Image / TF）

### 第二轮：单点自主导航

- 静态地图生成工具 `tools/generate_test_map.py`（Windows 可直接运行，从 `test.world` 碰撞几何生成 PGM + YAML）
- 定位 AMCL（差速运动模型，`map → odom → base_footprint`）
- 全局路径规划 `navfn/NavfnROS`，局部规划 `dwa_local_planner/DWAPlannerROS`
- 全局 / 局部代价地图（静态地图、激光障碍层、膨胀层），避障
- 目标点发送脚本 `scripts/send_goal.py`（Python 2.7/3 兼容）
- 一键启动 `nav_sim.launch` 与导航 RViz 配置 `navigation.rviz`

### 第三轮：全程航点导航与比赛主流程骨架

- 航点配置 `config/mission.yaml`（`task`/`transit`/`finish` 类型、坐标、有限重试、失败策略）
- 顺序航点导航 `scripts/mission_manager.py`（单一持久 move_base Action 客户端）
- 任务点接口（占位：打印提示 + 停留，识别/对准/击打未接入）
- 导航失败有限重试、超时取消确认、中断/ROS shutdown 处理
- 运行记录：每次运行写一份 JSON 到 `~/.ros/aigc/`（可覆盖目录）
- 共享模块 `src/aigc_navigation/navigation_client.py`（经 `setup.py` 安装），`send_goal.py` 与其复用同一套导航客户端
- 启动文件 `mission.launch`（仅任务管理器）与 `mission_sim.launch`（仿真 + 导航栈 + 任务管理器）

## 2. 工作方式

- **Windows 编写**：本工程在 Windows 上编写并提交到 Git。
- **Ubuntu 运行**：将代码同步到 Ubuntu 后编译、运行与测试。
- 文本文件统一使用 **UTF-8** 编码，源码/配置统一使用 **LF** 换行（见 `.gitattributes`），避免 Windows 换行影响 Ubuntu 使用。

## 3. 目标运行环境

- Ubuntu 18.04
- ROS1 Melodic
- Gazebo Classic 9

本轮**不**安装/运行 ROS、Gazebo、catkin，不安装 WSL/Docker，也不配置 Ubuntu 环境。

## 4. 工程目录

```text
catkin_ws/
  src/
    aigc_description/         # 机器人模型（URDF/xacro）
      urdf/robot.urdf.xacro
    aigc_sim/                 # 仿真启动、场地、RViz
      launch/sim.launch
      worlds/test.world
      rviz/sim.rviz
    aigc_navigation/          # 单点导航（第二轮）+ 全程航点导航（第三轮）
      launch/navigation.launch
      launch/nav_sim.launch
      launch/mission.launch      # 仅航点任务管理器（第三轮）
      launch/mission_sim.launch  # 仿真 + 导航栈 + 任务管理器（第三轮）
      config/                 # AMCL / move_base / costmap / DWA 参数
      config/mission.yaml     # 航点任务配置（第三轮）
      maps/test.yaml          # 静态地图（生成工具输出，已提交）
      maps/test.pgm
      rviz/navigation.rviz
      scripts/send_goal.py
      scripts/mission_manager.py   # 航点任务管理器（第三轮）
      src/aigc_navigation/        # 共享 Python 模块（第三轮）
      setup.py                    # catkin Python 模块安装（第三轮）
tools/
  generate_test_map.py        # 从 test.world 生成静态地图（Python 3 标准库）
docs/
  verification.md             # 验证记录
README.md
.gitignore
.gitattributes
```

`catkin_ws/src/CMakeLists.txt` 是 catkin 工作区顶层文件，由 Ubuntu 上的 catkin 初始化/构建时生成，本仓库不提交。

## 5. 模型参数（临时测试参数）

以下参数集中在 `catkin_ws/src/aigc_description/urdf/robot.urdf.xacro` 顶部，**均为临时测试参数**，待原机器人源码提供后再替换：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| 车身长/宽/高 | 0.28 / 0.20 / 0.06 m | 车身中心距地面 0.08 m |
| 轮半径 | 0.035 m | 轮径 0.07 m |
| 轮宽 | 0.02 m | |
| 轮距（左右轮中心距） | 0.16 m | |
| 支撑球半径 | 0.008 m | 前后各一个，低摩擦 |
| 相机安装位置 | x=0.14, y=0, z=0.08 m | 车身前部，朝 +x |
| 激光安装高度 | 0.20 m | 扫描平面距地面约 0.20 m |

## 6. 接口表

### 仿真 / 机器人

| 话题 / TF | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅：速度指令（差速） |
| `/odom` | `nav_msgs/Odometry` | 发布：里程计（`odometrySource=encoder` 编码器模式） |
| `odom → base_footprint` | TF | 里程计 TF，由差速插件发布 |
| `/joint_states` | `sensor_msgs/JointState` | 轮关节状态 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达，frame_id=`laser_link` |
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 相机图像，frame_id=`camera_optical_frame` |
| `/usb_cam/camera_info` | `sensor_msgs/CameraInfo` | 相机内参，frame_id=`camera_optical_frame` |
| `base_footprint → base_link` | TF | 固定关节，由 robot_state_publisher 发布 |
| `base_link → laser_link` | TF | 固定关节 |
| `base_link → camera_optical_frame` | TF | 固定关节 |

### 导航

| 话题 / TF | 类型 | 说明 |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图（map_server 发布） |
| `map → odom` | TF | 定位结果，由 AMCL 发布 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅：2D 初始位姿估计（RViz） |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 订阅：2D 导航目标（RViz） |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | move_base Action 目标 |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | move_base Action 反馈 |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | move_base Action 结果 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | move_base Action 状态 |
| `/move_base/NavfnROS/plan` | `nav_msgs/Path` | 全局路径 |
| `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | 局部路径 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 全局代价地图 |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 局部代价地图 |

坐标系约定（REP-103）：`base_link` 的 x 朝前、y 朝左、z 朝上；`camera_optical_frame` 的 z 朝前、x 朝右、y 朝下。

插件参数说明：三个 Gazebo 插件均使用 ROS1 Melodic 的驼峰参数名（如 `commandTopic`、`cameraName`、`topicName`）；差速插件显式配置 `odometrySource=encoder`；相机/雷达的 `update_rate` 是 SDF sensor 原生参数，频率由 sensor 控制。

## 7. Ubuntu 依赖安装

以下命令假定 Ubuntu 已安装 ROS Melodic 且 apt 源可用（本轮不展开 ROS 安装教程）。

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-melodic-catkin \
  ros-melodic-xacro \
  ros-melodic-robot-state-publisher \
  ros-melodic-gazebo-ros \
  ros-melodic-gazebo-plugins \
  ros-melodic-rviz \
  ros-melodic-urdf \
  ros-melodic-tf \
  ros-melodic-rospy \
  ros-melodic-navigation \
  ros-melodic-move-base-msgs \
  ros-melodic-actionlib
```

`ros-melodic-navigation` 是导航元包，包含 `map_server`、`amcl`、`move_base`、`navfn`、`dwa_local_planner`、`clear_costmap_recovery`、`rotate_recovery`、`costmap_2d` 等；`ros-melodic-move-base-msgs`、`ros-melodic-actionlib` 供 `send_goal.py` 使用。`ros-melodic-urdf` 提供 `check_urdf`；`ros-melodic-tf` 提供 `tf_echo` 等 TF 检查工具；`ros-melodic-rospy` 用于运动测试脚本（通常随 ROS 基础安装已包含）。

## 8. Ubuntu 编译与启动

从仓库根目录开始：

```bash
source /opt/ros/melodic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch aigc_sim sim.launch
```

不打开界面、只跑仿真：

```bash
roslaunch aigc_sim sim.launch gui:=false rviz:=false
```

> 每个新终端都需要重新 `source /opt/ros/melodic/setup.bash` 和 `source devel/setup.bash`。

## 9. 话题与 TF 检查

```bash
# 话题列表
rostopic list

# 频率（注意：Image 话题较大，用 hz 检查，不要完整 echo 打印）
rostopic hz /clock
rostopic hz /odom
rostopic hz /scan
rostopic hz /usb_cam/image_raw

# 消息头 / 元信息（大体积 Image 不直接完整打印）
rostopic info /odom
rostopic info /scan
rostopic info /usb_cam/image_raw
rostopic info /usb_cam/camera_info
rostopic echo -n1 /joint_states

# TF 检查
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint base_link
rosrun tf tf_echo base_link laser_link
rosrun tf tf_echo base_link camera_optical_frame
```

相机画面可在 RViz 的 Image 面板查看，或用 `rqt_image_view /usb_cam/image_raw`。

## 10. 低速运动测试

测试前**确保机器人前方留有空间**。本脚本不依赖差速插件一定具有速度指令超时保护，所有动作均有限时长，并在结束时重复发布零速度。

将下面的代码保存为 `motion_test.py` 后运行：

```bash
python motion_test.py
```

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
低速、有限时长的运动测试：
依次执行 前进 -> 停车 -> 后退 -> 停车 -> 原地左转 -> 停车 -> 原地右转 -> 停车。
每个动作固定时长，结束时重复发布零速度确保停车。
"""
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('aigc_motion_test')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.sleep(1.0)  # 等待话题连接

    zero = Twist()

    # (线速度 m/s, 角速度 rad/s, 时长 s, 说明)
    motions = [
        (0.10, 0.0, 3.0, 'forward'),
        (0.0, 0.0, 1.0, 'stop'),
        (-0.10, 0.0, 3.0, 'backward'),
        (0.0, 0.0, 1.0, 'stop'),
        (0.0, 0.5, 3.0, 'turn left in place'),
        (0.0, 0.0, 1.0, 'stop'),
        (0.0, -0.5, 3.0, 'turn right in place'),
        (0.0, 0.0, 1.0, 'stop'),
    ]

    rate = rospy.Rate(20.0)
    try:
        for lin, ang, duration, label in motions:
            rospy.loginfo("motion: %s (lin=%.2f ang=%.2f for %.1fs)",
                          label, lin, ang, duration)
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            end = rospy.Time.now() + rospy.Duration(duration)
            while rospy.Time.now() < end:
                pub.publish(msg)
                rate.sleep()
    finally:
        # 结束时重复发布零速度，确保停车
        for _ in range(10):
            pub.publish(zero)
            rospy.sleep(0.05)


if __name__ == '__main__':
    main()
```

## 11. 单点自主导航（第二轮）

### 11.1 地图生成（Windows 可执行）

```bash
python tools/generate_test_map.py
```

脚本只使用 Python 3 标准库（无需 ROS / numpy / OpenCV），解析 `test.world` 的静态 box 碰撞几何，输出 `catkin_ws/src/aigc_navigation/maps/test.pgm` 与 `test.yaml`。地面不作为障碍物。生成结果已提交，Ubuntu 上可直接使用，无需重新生成。

地图参数：200×200 像素，分辨率 0.02 m/像素，原点 `[-2.0, -2.0, 0.0]`（覆盖 x/y ∈ [-2.0, 2.0]）。

### 11.2 启动导航（Ubuntu）

```bash
source /opt/ros/melodic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash

# 一键：Gazebo 仿真 + 导航栈 + 导航 RViz
roslaunch aigc_navigation nav_sim.launch

# 或：无界面仿真 + 无 RViz
roslaunch aigc_navigation nav_sim.launch gui:=false rviz:=false
```

`nav_sim.launch` 内部包含 `sim.launch`（关闭其 RViz，避免重复开窗口）与 `navigation.launch`（map_server + AMCL + move_base + 可选导航 RViz）。若仿真已在运行，也可单独启动导航栈：

```bash
roslaunch aigc_navigation navigation.launch
```

### 11.3 发送目标点

```bash
# 默认目标 (1.2, 0, 0)
rosrun aigc_navigation send_goal.py

# 指定目标（x, y, 偏航角 rad）
rosrun aigc_navigation send_goal.py _x:=1.2 _y:=0.0 _yaw:=0.0
```

`send_goal.py`（Python 2.7/3 兼容）通过 move_base Action 发送单个目标，只有动作结果为 `SUCCEEDED` 才算成功；超时、中止、拒绝均以非零退出码退出。退出或 Ctrl+C 时只取消**自己**发送的目标（不调用 `cancel_all_goals` 干扰其他客户端），并发布零速度停车。也可在 RViz 中用「2D Nav Goal」按钮点击目标点。

### 11.4 导航参数

- **定位**：AMCL，差速运动模型，帧 `map → odom → base_footprint`，初始位姿 (0, 0, 0)，粒子数 500～2000。
- **全局规划**：`navfn/NavfnROS`，静态地图层 + 激光障碍层 + 膨胀层，规划频率约 1 Hz。
- **局部规划**：`dwa_local_planner/DWAPlannerROS`，激光障碍层 + 膨胀层（odom 下滚动窗口 3 m × 3 m），控制频率约 5 Hz，最大前进速度 0.15 m/s，横向速度为零。
- **代价地图**：机器人 footprint 为覆盖车身/车轮/相机的最小矩形 `[[-0.16,-0.11],[-0.16,0.11],[0.17,0.11],[0.17,-0.11]]`，膨胀半径 0.30 m。
- **恢复行为**：保守重置、积极重置、原地旋转。

### 11.5 注意事项

- 导航测试时**不要**同时运行第 10 节的运动测试脚本或其他向 `/cmd_vel` 发布速度的程序，避免多个控制器争用 `/cmd_vel`。
- 当前场地为静态；`test_object` 是固定障碍物，全局/局部路径会自动绕开它。

## 12. 全程航点导航与比赛主流程（第三轮）

### 12.1 主流程

`mission_manager.py` 按 `config/mission.yaml` 的航点顺序依次导航：

1. 初始化并校验配置；
2. 等待导航就绪（move_base 服务连接 + `map → base_footprint` TF + 仿真时钟启动；连接等待以 `server_timeout` 为上限、TF 用非阻塞 `canTransform` 轮询，全程以 `ready_timeout` 墙上时间为界、可中断）；
3. 用单一持久 Action 客户端顺序导航每个航点，失败按 `max_retries` 有限重试；
4. 到达 `task` 航点后调用任务接口（当前为占位）；
5. 到达 `finish` 后结束，保存一次运行的 JSON 记录并退出。

航点类型：`task`（任务点，到达后调用任务接口）、`transit`（中转点，失败即停止）、`finish`（终点，必须最后，失败不算完成）。

### 12.2 航点配置

`config/mission.yaml` 在 launch 时通过 `<rosparam>` 加载到节点私有命名空间。主要参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `frame_id` | `map` | 目标坐标系 |
| `ready_timeout` / `server_timeout` / `goal_timeout` / `cancel_timeout` | 30 / 10 / 60 / 3 | 均为墙上秒 |
| `max_retries` | 1 | 首次失败后最多重试次数（共 2 次尝试），非负整数 |
| `failure_policy` | `stop` | 任务点失败策略：`stop` 停止 / `skip` 跳过（最终 PARTIAL） |
| `dwell_time` | 0.5 | 到达任务点后的停留秒数 |
| `waypoints` | 见文件 | 有序列表，每项 `name`/`kind`/`x`/`y`/`yaw`，`finish` 必须最后 |

修改 `mission.yaml` 后重新 `roslaunch` 即可生效。

### 12.3 运行记录

每次启动生成新的 `run_id`，在 `~/.ros/aigc/`（可用 `log_dir` 参数覆盖）写一份 JSON，记录 `run_id`、起止时间、最终状态、退出码、耗时，以及每个航点的名称/坐标/尝试次数/结果/原因。成功、失败、中断时都会保存，且**不覆盖**旧记录；保存失败不会把导航失败改写为成功。

### 12.4 退出码

| 退出码 | 含义 |
| --- | --- |
| 0 | 全部完成（COMPLETED） |
| 1 | 配置无效或导航未就绪 |
| 2 | 导航失败 / 超时 / 取消未确认 / 连接丢失 |
| 3 | 跳过了任务点（PARTIAL） |
| 130 | 中断（Ctrl+C）或 ROS shutdown |

### 12.5 启动

```bash
source /opt/ros/melodic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash

# 一键：Gazebo 仿真 + 导航栈 + 任务管理器
roslaunch aigc_navigation mission_sim.launch

# 无界面 / 无 RViz
roslaunch aigc_navigation mission_sim.launch gui:=false rviz:=false

# 仅任务管理器（仿真与导航栈已在运行时）
roslaunch aigc_navigation mission.launch
```

`mission.launch` 只启动 `mission_manager` 节点并加载 `mission.yaml`；`mission_sim.launch` 包含 `nav_sim.launch` 与 `mission.launch`，支持 `gui`/`rviz`/`paused` 及 `mission_file`/`log_dir` 参数。**不要**同时运行 `mission.launch` 与 `send_goal.py`（会争用同一 `/cmd_vel`）。

### 12.6 与 send_goal.py 的关系

`mission_manager.py` 复用 `send_goal.py` 的 Action 接口、墙上时间超时、shutdown 检查、终态判断与取消停车，这些逻辑提取到共享模块 `src/aigc_navigation/navigation_client.py`（经 `setup.py` + `catkin_python_setup()` 安装）。`send_goal.py` 命令仍可单独使用。

多点巡航的调用方式**参考**原 `abot_patrol_nav.py` 巡航示例（`SimpleActionClient("move_base", MoveBaseAction)` + 保存目标列表 + 顺序发送 + 读取动作结果），**未迁移整个原源码包**。

### 12.7 就绪检查与目标生命周期（修复说明）

上一版文档称“就绪检查完全受实际时间限制”，但当时的 TF 就绪用 `waitForTransform(..., rospy.Duration(0.2))`，其超时依赖 ROS 时间，`/clock` 暂停时可能阻塞，该结论不准确。本版已修正：

- 就绪检查改为非阻塞 `canTransform` 轮询 + 普通 `time.sleep`，服务连接等待 ≤ `server_timeout`、就绪总时长 ≤ `ready_timeout`，全程与 `/clock` 无关、可中断；就绪失败/中断时零发送。
- 目标所有权只在确定终态（SUCCEEDED/ABORTED/REJECTED/PREEMPTED/RECALLED）释放；LOST/取消超时/异常/shutdown 保留所有权，清理可重试取消；旧目标未确认时不发送新目标。
- 取消确认期间 shutdown 视为中断（退出 130、状态 STOPPED），与普通“取消未确认”（退出 2）区分；`kind` 字段必填且限 `task`/`transit`/`finish`。

## 13. 尚未完成的功能

- 视觉识别（模板/AR）、对准、击打（任务接口当前为占位）
- 大模型服务
- 正式比赛场地与真实机器人模型（当前为临时测试参数与基础几何体）

## 14. 待 Ubuntu 验证的内容

本轮在 Windows 上仅做了静态检查（XML/YAML 语法、脚本编译、文件引用与参数一致性、替身分支检查、航点碰撞核对等），**尚未在 Ubuntu 上实际运行**。以下内容待 Ubuntu 验证，详见 `docs/verification.md`：

- `xacro` 展开、`check_urdf`
- `catkin_make` 编译（含 `catkin_python_setup()`/`setup.py` 安装共享模块）
- Gazebo 启动与机器人出生/稳定
- 话题与频率（`/odom`、`/scan`、`/usb_cam/image_raw` 等）
- TF 树
- 相机画面、雷达测距
- 前进、后退、原地转向、停车
- 地图被 map_server 正确加载
- AMCL 定位收敛（`map → odom` 稳定）
- move_base 规划出全局路径、局部代价地图/避障生效
- `send_goal.py` 导航到目标点并返回 `SUCCEEDED`
- `mission_manager` 依次导航全部航点并到达 `finish`（退出码 0、记录 COMPLETED）
- 航点失败重试与 `failure_policy` 停止/跳过、超时取消确认、中断退出 130、运行记录写入与不覆盖
