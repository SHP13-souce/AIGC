# 验证记录

本轮范围：机器人模型、测试场地、仿真启动文件、传感器配置、运行说明。
本轮在 Windows 上完成静态检查；Ubuntu 上的编译与运行验证留待后续执行。

起始 commit：`a7fd986bd06be67a02fbde3d73cfe0256edc1fb9`

---

## A. Windows 已实际执行的检查

在 Windows（无 ROS/Gazebo）上执行，仅使用已安装的 Python 3.13 标准库。

| # | 检查项 | 命令 / 方法 | 结果 |
| --- | --- | --- | --- |
| 1 | 所有 XML 文件语法 | Python `xml.etree.ElementTree` 逐个解析 | 通过（见下方命令） |
| 2 | 包名 / 依赖 / CMake | 人工核对 package.xml 与 CMakeLists.txt | 通过 |
| 3 | launch 引用文件存在 | 检查 `sim.launch` 引用的 world/rviz/xacro 路径 | 通过 |
| 4 | 无个人电脑绝对路径 | 全文搜索 `C:\`、`/home/`、`/Users/` | 通过 |
| 5 | 轮关节名一致 | 比对 xacro 关节名与差速插件 `left_joint`/`right_joint` | 通过 |
| 6 | 轮距 / 轮径一致 | 比对 xacro 属性与插件 `wheel_separation`/`wheel_diameter` | 通过 |
| 7 | 传感器话题 / frame_id 一致 | 比对插件参数与接口表 | 通过 |
| 8 | TF 无重复发布 | 插件 `publish_wheel_tf=false`，轮 TF 由 robot_state_publisher 发布 | 通过 |
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

## B. 静态审查结果

- **模型**：使用基础几何体（box/cylinder/sphere），无外部 STL、无在线下载；单位统一为米/千克/弧度；实体均有 visual、collision、质量和惯量。
- **驱动轮**：圆柱绕 y 轴（`rpy="1.5708 0 0"`），关节轴 `0 1 0`，连续关节；左轮 `+y`、右轮 `-y`；驱动轮接地（轮心 z=0.035=半径）。
- **支撑**：前后各一个低摩擦球（`mu1/mu2=0.01`），不阻碍转向。
- **差速插件**：`libgazebo_ros_diff_drive.so`，`command_topic=cmd_vel`、`odometry_topic=odom`、`odometry_frame=odom`、`robot_base_frame=base_footprint`，编码器里程计模式；`publish_odom_tf=true` 发布 `odom → base_footprint`；`publish_wheel_tf=false`，轮 TF 交给 robot_state_publisher；`publish_wheel_joint_state=true`。
- **相机**：`libgazebo_ros_camera.so`，640×480、15 Hz、`camera_name=usb_cam`，话题 `/usb_cam/image_raw`、`/usb_cam/camera_info`，`frame_name=camera_optical_frame`；optical frame 通过固定关节 `rpy="-1.5708 0 -1.5708"` 得到 z 前、x 右、y 下。
- **激光**：`libgazebo_ros_laser.so`，360°、360 采样、10 Hz、`frame_id=laser_link`；扫描平面距地面约 0.20 m（墙高 0.30 m 可被扫到）。
- **场地**：地面与光照均内联；围墙内净尺寸 3.6 m × 3.6 m（内侧面 ±1.8 m），墙厚 0.10 m，墙高 0.30 m；前方 0.8 m 处有彩色测试物体；机器人出生点 (0,0) 与场地、物体不重叠。
- **启动**：`use_sim_time=true`、加载 world、xacro 生成 `robot_description`、spawn 机器人（z=0.02 防穿地）、robot_state_publisher、可选 RViz；支持 `gui`/`rviz`/`paused` 参数，默认不暂停。
- **RViz**：Fixed Frame=`odom`，显示 RobotModel、LaserScan（`/scan`）、Image（`/usb_cam/image_raw`）、TF；不创建 map 坐标系，无额外静态 TF。

## C. 待 Ubuntu 执行的验证

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
