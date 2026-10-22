> **最新实测（2026-10-22）**：运行 `aigc_20261022T085545_bc104676`，82.23秒，COMPLETED/0。官方识别返回数学1、榴莲6；模板113、AR1、AR6及终点均一次成功。三个任务完成对准和模拟射击，object_kp=0.002。起终点语音消息均已发布，实际听音待确认。本次覆盖一个样例和测试地图；真实发射、命中及正式场地尚待验证。下文“尚未联调”等早期状态以本条更新为准。

# AIGC 无人车仿真底座

无人车挑战赛的仿真底座与 ROS1 任务主程序，目前包含：

- **第一轮**：最小仿真底座——可运动的差速机器人、测试场地、相机与激光雷达。
- **第二轮**：单点自主导航——基于已有 ROS1 导航包的定位（AMCL）、全局路径规划（navfn）、局部规划（DWA）与避障，支持导航到单个目标点。
- **第三轮**：全程航点导航与比赛主流程骨架——按配置顺序导航多个航点，到达任务点调用任务接口，含失败重试、超时/中断处理与 JSON 运行记录。
- **第四轮**：AR 测试靶识别、对准与模拟击打——接入官方 ar_track_alvar 识别测试靶、按方位误差原地对准、调用 `/shoot` 接口模拟击打，串联现有比赛流程。
- **赛方接口适配**：新增任务识别触发、数学/水果双结果选靶、模板像素对准与语音播报；单独的 competition 入口使用独立模拟射击话题，尚待 Ubuntu 上与赛方节点实测。

> 原测试场景使用 **AR 测试靶**（9×9 图案，**非正式比赛靶**）。新增适配器可调用赛方已有双结果识别服务，不复制或重实现云识别客户端；该接口实际效果仍待联调。**正式比赛场地、正式靶与真实机器人模型尚未验收**。击打为进程内**模拟**（无弹道/射线命中检测，`hit` 恒为 `null`，不声称“实测命中”）。

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
    aigc_sim/                 # 仿真启动、场地、RViz、测试靶资源
      launch/sim.launch
      worlds/test.world
      rviz/sim.rviz
      meshes/marker_0.dae      # AR 测试靶白板+图案网格（第四轮）
      meshes/marker_1.dae
      meshes/marker_2.dae
      materials/textures/marker_0.png   # 180×180 灰度测试图案（第四轮）
      materials/textures/marker_1.png
      materials/textures/marker_2.png
    aigc_navigation/          # 单点导航（第二轮）+ 全程航点导航（第三轮）+ AR 任务（第四轮）
      launch/navigation.launch
      launch/nav_sim.launch
      launch/mission.launch      # 仅航点任务管理器（第三轮）
      launch/mission_sim.launch  # 仿真 + 导航栈 + 任务管理器（第三轮）
      launch/ar_detector.launch      # ar_track_alvar 测试靶检测（第四轮）
      launch/task_mission_sim.launch # 仿真 + 导航 + AR + 任务（第四轮）
      config/                 # AMCL / move_base / costmap / DWA 参数
      config/mission.yaml     # 航点任务配置（第三轮，第四轮加 execute_tasks/target_id）
      config/task.yaml        # AR 任务参数（第四轮）
      maps/test.yaml          # 静态地图（生成工具输出，已提交，第四轮含测试靶障碍）
      maps/test.pgm
      rviz/navigation.rviz
      scripts/send_goal.py
      scripts/mission_manager.py   # 航点任务管理器（第三轮，第四轮接入任务）
      src/aigc_navigation/        # 共享 Python 模块（第三轮起）
        navigation_client.py      # 共享导航客户端
        sim_shooter.py            # 模拟击打后端（第四轮）
        task_executor.py          # 识别/对准/击打任务执行（第四轮）
      setup.py                    # catkin Python 模块安装（第三轮）
tools/
  generate_test_map.py        # 从 test.world 生成静态地图（Python 3 标准库）
  generate_test_markers.py    # 生成测试靶灰度 PNG（Python 3 标准库，第四轮）
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

### AR 测试靶任务（第四轮）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/ar_pose_marker` | `ar_track_alvar_msgs/AlvarMarkers` | ar_track_alvar 检测结果，`output_frame=camera_optical_frame` |
| `/shoot` | `std_msgs/String` | 击打接口，命令 `shoot` / `stopshoot`（本轮由模拟后端收发） |

> AR 检测使用官方 `ar_track_alvar` 的 `individualMarkersNoKinect`（**不重实现检测、不改为 OpenCV ArUco**）。每个 marker 的**自身 header**（`marker.header.stamp` + `marker.header.frame_id`）才是权威来源，**不读取**容器消息 header 或 `pose.header`。

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
  ros-melodic-actionlib \
  ros-melodic-ar-track-alvar \
  ros-melodic-ar-track-alvar-msgs
```

`ros-melodic-navigation` 是导航元包，包含 `map_server`、`amcl`、`move_base`、`navfn`、`dwa_local_planner`、`clear_costmap_recovery`、`rotate_recovery`、`costmap_2d` 等；`ros-melodic-move-base-msgs`、`ros-melodic-actionlib` 供 `send_goal.py` 使用；`ros-melodic-ar-track-alvar` 与 `ros-melodic-ar-track-alvar-msgs` 供第四轮 AR 检测使用（只依赖官方包，不复制其源码）。`ros-melodic-urdf` 提供 `check_urdf`；`ros-melodic-tf` 提供 `tf_echo` 等 TF 检查工具；`ros-melodic-rospy` 用于运动测试脚本（通常随 ROS 基础安装已包含）。

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
| `execute_tasks` | `false` | `true` 时在任务点导航成功后执行 AR 识别/对准/模拟击打；`false` 时保持原纯导航流程 |
| `waypoints` | 见文件 | 有序列表，每项 `name`/`kind`/`x`/`y`/`yaw`；`task` 项可含 `target_id`（0/1/2，缺省按 `task_1/2/3 → 0/1/2` 映射）；`finish` 必须最后 |

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

## 13. AR 测试靶任务（第四轮）

本轮把“识别、对准、击打”接入现有比赛流程：导航到任务点成功后，用官方 `ar_track_alvar` 识别**测试靶**（9×9 图案），按方位误差原地对准，再通过 `/shoot` 接口做一次**模拟**击打，最后到达 `finish` 保存结果。

### 13.1 主流程（`execute_tasks: true`）

`task_mission_sim.launch` 将 `mission_manager` 的 `execute_tasks` 置为 `true`。对每个 `task` 航点：

1. `move_base` 导航成功后**先停车并等待车身稳定**（读取 `/odom`，线/角速度低于阈值，受 `settle_timeout` 与整项任务期限双重约束）；
2. `task_executor.py` 原地旋转搜索目标：无检测时以 `search_rate` 低速搜索，检测到后按 `error = atan2(x, z)` 比例转向（`angular.z = clamp(-kp·error)`，仅原地、无平移）；
3. **对准与停车稳定合并判定**：只有「对准（`|error| ≤ align_tolerance`）且车身已停稳」的帧才累计稳定帧，只对**严格递增**的新图像时间戳计数（同一帧重复读取与倒退帧都不计数也不清零，A、B、A 不会累计成 3）；连续 `stable_frames` 帧即认为可击打；
4. **击打前最终门槛**：在触发 `sim_shooter.py` 的瞬间重新确认——任务仍有效且未 shutdown、整项任务未超时、目标仍新鲜且对准、新鲜里程计表明已停稳、稳定帧条件仍成立、该点击打额度未消耗；任一不成立则重新寻找/对准或明确返回失败/超时，**绝不击打**；
5. 成功后继续下一任务点；失败按 `failure_policy` 停止/跳过；**任意阶段中断 → 退出 130**、超时 → 超时（两者都**不**记为成功）；每点每轮**最多一次**模拟击打，额度在触发前预留（异常不会重复触发），前次失败/超时/中断不会被「已经打过」改写成成功。

整项任务共用**同一个绝对墙上期限**（`task_timeout`），覆盖停车、搜索、对准、再次稳定、模拟脉冲与反馈等待；各阶段开始与每次等待返回后都检查 shutdown 与期限。所有等待均使用墙上时间，`/clock` 暂停不能使任务卡死。

### 13.2 测试靶资源

- 三个白色靶板（0.20 × 0.20 × 0.02 m）作为静态障碍加入 `test.world`，黑色图案 0.09 m、中心高 0.15 m：ID0 中心 (1.2, 0.65, 0.15) 朝 -y，ID1 (0.55, 1.0, 0.15) 朝 +x，ID2 (-1.0, 0.35, 0.15) 朝 +y。
- 图案为 180×180 无损灰度 PNG（`tools/generate_test_markers.py`，Python 3 标准库生成并已提交），配 COLLADA 网格 `meshes/marker_*.dae`（0.09 m 平面、UV 映射、白色背板碰撞）；网格以 `model://aigc_sim/meshes/...` 引用。
- **资源搜索路径**：`model://aigc_sim/...` 通过 **`gazebo_model_path`** 解析（指向 `GAZEBO_MODEL_PATH`，搜索根目录为 `${prefix}/..`，源空间与安装空间下都能命中 `aigc_sim/meshes/marker_N.dae`）；`gazebo_media_path`（`GAZEBO_RESOURCE_PATH`）仅用于 DAE 的相对纹理引用（`../materials/textures/*.png`），两者并存、各司其职。上一版只用 `gazebo_media_path` 来解析 `model://` 网格引用是错误的，已修正。
- 三块靶板已纳入静态地图（`test.pgm`/`test.yaml` 重新生成并提交）。

### 13.3 AR 检测（`ar_detector.launch`）

使用官方 `ar_track_alvar` 的 `individualMarkersNoKinect`（**不复制源码、不重实现检测、不改为 OpenCV ArUco**）：

- `marker_size=9.0`（cm）、`marker_resolution=5`、`marker_margin=2`、`output_frame=camera_optical_frame`；
- 重映射 `camera_image → /usb_cam/image_raw`、`camera_info → /usb_cam/camera_info`；
- 结果发布到 `/ar_pose_marker`（`ar_track_alvar_msgs/AlvarMarkers`），按 marker 的**自身 header**（`stamp` 为图像时间戳、`frame_id` 为输出帧）读取。

### 13.4 检测稳健性（`task_executor.py`）

- 任务开始时记录任务开始的 ROS 时间并清空计数与缓存；有效检测至少要求：marker 自身 header 时间戳 **大于零**且**在任务开始之后**采集（拒绝零时间戳、任务前图像），图像时间戳年龄与消息墙上接收年龄都合格，ID/frame/坐标合法；
- 按 `target_id` 过滤；同 id 多个 marker 时不盲取；坐标必须有限且 `z > 0`；`frame_id` 必须等于 `output_frame`；
- 双重新鲜度：图像时间戳年龄 ≤ `detection_max_age`，且消息收到的墙上年龄 ≤ `receive_max_age`（`/clock` 暂停会使检测失效）；
- 稳定计数只对**严格递增**的新图像时间戳增加：同一帧重复读取不增加也不误判丢失，倒退帧不计数（A、B、A 不会累计成 3）；真正丢失、过期或偏差超限才清零；
- 停稳判定同时检查里程计消息的墙上接收年龄与 **`odom.header.stamp` 的有效性和年龄**：仅凭「刚收到」不能把旧里程计当成新数据；未停稳不击打；任务结束时发布有限次零速度并结束模拟脉冲。

### 13.5 模拟击打（`sim_shooter.py`）

进程内模拟，沿用原击打接口 `/shoot`（`std_msgs/String`，命令 `shoot` / `stopshoot`）：发一次 `shoot` 再发 `stopshoot`，可中断；反馈含 `run_id/waypoint/target_id/shot_id/backend=simulated/success/reason`，`hit` 恒为 `null`。结果由 `task.yaml` 的 `shooter.success` 配置（`false` 可模拟击打失败）。**不代表真实命中**。

模拟后端接受整项任务的绝对期限：调用开始时已 shutdown 或超时则**不发布 `shoot`**；脉冲与反馈等待都遵守期限；用 `try/finally` 结束已开始的脉冲；中断返回 `interrupted`、超时返回 `timeout`，两者都**不**沿用配置的 `success=true`（清理允许有限耗时，但超时后不报告业务成功）。

### 13.6 任务参数（`config/task.yaml`）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `output_frame` | `camera_optical_frame` | 检测结果坐标系 |
| `control_rate` / `kp` / `max_angular_rate` | 20 / 1.0 / 0.35 | 对准控制环（Hz / 比例增益 / 角速度上限 rad/s） |
| `search_rate` | 0.15 | 无目标时的低速搜索角速度 rad/s |
| `align_tolerance` / `stable_frames` | 0.035 / 3 | 对准阈值 rad / 稳定帧数 |
| `detection_max_age` / `receive_max_age` | 0.3 / 0.5 | 图像时间戳 / 墙上接收年龄上限 s |
| `task_timeout` / `settle_timeout` | 20 / 3 | 整个任务（停车+搜索+对准+再次稳定+脉冲+反馈）/ 停车稳定超时 s |
| `odom_max_age` / `odom_max_linear` / `odom_max_angular` | 0.5 / 0.05 / 0.05 | 稳定判定阈值 |
| `shooter` | 见文件 | 模拟击打后端（topic/脉冲/延迟/结果） |

以上均为**测试初值**，非正式比赛参数。

### 13.8 任务参数校验（仅在 `execute_tasks=true` 时、任何转向/击打之前）

启动后先校验任务参数，任一不合法即打印明确错误、退出码 1，且**不发布任何转向或击打指令**：`execute_tasks` 与 `shooter.success` 必须为布尔；`control_rate`/`task_timeout`/各类年龄、速度/角度/停稳阈值必须为正值有限数；`stable_frames`/`stop_publish_count` 必须为正整数（拒绝布尔）；`search_rate ≤ max_angular_rate`；`pulse_duration` 为正、`delay` 非负有限；话题名与 `output_frame` 必须为非空字符串。

### 13.7 启动（Ubuntu）

```bash
source /opt/ros/melodic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash

# 一键：仿真 + 导航栈 + AR 检测 + 任务管理器（识别/对准/模拟击打）
roslaunch aigc_navigation task_mission_sim.launch

# 无界面 / 无 RViz
roslaunch aigc_navigation task_mission_sim.launch gui:=false rviz:=false
```

原第三轮启动方式不变：`mission_sim.launch`（`execute_tasks=false`，纯导航）与 `send_goal.py` 仍可单独使用。`task_mission_sim.launch` 内部包含 `nav_sim.launch`（仿真 + 导航栈）与 `ar_detector.launch`，Gazebo/导航/RViz **不重复启动**。

## 14. 尚未完成的功能

- 大模型服务
- 正式比赛场地、正式比赛靶与真实机器人模型（当前为临时测试参数、测试靶与基础几何体）
- 真实击打硬件/串口驱动（当前为进程内模拟，无弹道/射线命中检测）

## 15. 待 Ubuntu 验证的内容

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
- AR 测试靶纹理在 Gazebo 中正确显示（无镜像/倒置，检测方向正确）
- `model://aigc_sim/meshes/...` 在源空间与安装空间都能被 `gazebo_model_path` 解析到（**尚未验证**，当前只确认了目录结构；DAE/XML 语法解析通过**不等于** Gazebo 正确显示）
- `ar_track_alvar` 识别到 ID0/1/2，`/ar_pose_marker` 的 marker 自身 header（stamp/frame_id）正确
- 检测双重新鲜度、同帧不重复计数、目标丢失清零、转向符号（偏右 → 负角速度）在实车上成立
- 对准稳定计数与模拟击打反馈、每点最多一次击打、模拟失败不记为任务成功
- 整项任务统一期限（含模拟脉冲/反馈）、击打前最终门槛（目标/里程计/稳定帧/额度复核）、中断退出 130、任务参数校验（非法配置退出 1 且无转向/击打）
- `execute_tasks=true` 三点任务 + `finish` 全流程（退出码 0、记录 COMPLETED）；`execute_tasks=false` 原纯导航流程不回归

# 赛方接口适配入口（2026-10-22）

新增 `competition_mission.launch`，连接赛方 `/detect` 参数、
`/fruit_detection` 双结果服务、`/object_position` 模板定位、动态 AR 选靶及语音。
射击仍为独立话题上的模拟反馈；尚未完成 Ubuntu 上的赛方整套代码联调。
原有测试场景入口保持可用。编译、接口检查、配置要求及源码版本差异见
[赛方接口适配说明](docs/competition_integration.md)。
