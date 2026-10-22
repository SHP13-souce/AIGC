> **最新实测（2026-10-22）**：运行 `aigc_20261022T085545_bc104676`，82.23秒，COMPLETED/0。官方识别返回数学1、榴莲6；模板113、AR1、AR6及终点均一次成功。三个任务完成对准和模拟射击，object_kp=0.002。起终点语音消息均已发布，实际听音待确认。本次覆盖一个样例和测试地图；真实发射、命中及正式场地尚待验证。下文“尚未联调”等早期状态以本条更新为准。

# 2026-10-22 官方接口实测与整段联调准备

环境：Ubuntu 18.04、ROS1 Melodic、Gazebo Classic；Windows 为宿主机。
本文的场景和坐标仅用于 AIGC `test.world`，不是正式赛场标定结果。

## 已由用户终端验证的结果

- Python 3.7.5 独立环境全部导入通过：rospy、ROS 消息、OpenCV、NumPy、Pillow、OpenAI。
- 官方 `robot_voice/tts_subscribe.cpp` 成功接收 `/voiceWords`、合成并播放语音。
  可执行文件位于 `~/.ros/aigc_voice/tts_subscribe`；补装了 mplayer。
- 官方模板检测约 5 Hz；相机拍摄同心圆模板后连续返回 ID 113。
  删除同心圆测试板后连续返回 `(0,0,255)`，未继续输出旧坐标。
- 官方 `identify.py` 调用自带的真实相机样例，返回数学答案 1、榴莲编码 6。
- 把该图片放入 Gazebo 后，真实 `/usb_cam/image_raw` 取图，经 `/detect` 参数触发，
  `/fruit_detection` 服务返回 `1,6`，触发参数复位到 255。
- 尚未执行本轮模板→数学 AR→水果 AR→终点的完整主程序验证；不记作通过。
- 本轮射击仍为独立话题的模拟脉冲，未验证实车发射、完成反馈或命中。

## 官方代码保持不变时的环境处理

- 原路径 `abot——01` 含 Unicode 长横线。使用 `~/.ros/aigc_vendor/ws` 软链接作为
  工作区入口，并让 ROS_PACKAGE_PATH、CMAKE_PREFIX_PATH 使用英文路径。
- 解压后的两个检测二进制曾缺执行权限，仅为这两个文件增加 `u+x`。
- 提供的源码在 `gui=false` 分支中连接了不匹配的 Qt 检测槽函数签名。
  当前用官方默认 `gui=true` 成功运行，不修改其算法或源文件。
- 曾出现重复节点名 `/print_objects_detected` 触发关闭；当前独立启动官方程序：

  ```bash
  source /tmp/aigc_vendor_env.sh
  rosrun find_object_2d print_objects_detected __name:=aigc_object_position
  ```

- 不要重复启动多个同名检测节点、旧任务示例或对准控制器。
- `/detect` 是参数；官方脚本日志中的同名话题触发提示与源码不符。
- 官方取图路径固定为 `/home/abot/abot_ws/src/abot_vlm/temp2/vl_now.jpg`，已创建
  对应可写目录。原厂样例在选定的官方工作区中，二者不是同一个文件。
- API 密钥仍由官方 `API_KEY.py` 加载；不复制到 AIGC、不打印、不提交。

当前安装版本（只用于独立识别环境，系统 Python 默认值保持原样）：

```text
Python 3.7.5; pip 24.0; setuptools 67.8.0; wheel 0.41.3
numpy 1.21.6; opencv-python 4.8.1.78; Pillow 9.5.0
rospkg 1.5.1; catkin-pkg 1.0.0; PyYAML 6.0.1
openai 1.10.0; httpx 0.24.1; pydantic 1.10.13
```

## 准备整段仿真检查

在 AIGC 仓库根目录运行：

```bash
source /tmp/aigc_vendor_env.sh
"$HOME/.venvs/aigc-vlm/bin/python" tools/prepare_vendor_sim_check.py
```

脚本先核对 Gazebo 中原有三个靶板的位置，拒绝其他场景；随后只增添三个命名测试
模型，不发底盘速度、不启动任务。它保留现有碰撞几何与地图，新增板均为视觉模型：

- 第一任务区：用官方同心圆模板 113 覆盖原 AR0 图案。
- 第二任务区：使用原场景的 AR1（本次样例数学答案）。
- 第三任务区：用 AR6 覆盖原 AR2 图案（本次样例榴莲编码）。
- 起始相机前：用官方样例照片作为识别测试板材质。

AR6 图案与原有 AR0/1/2 一样来自上游 ALVAR 的 `Markers_0_to_8.png`，采用
9×9 网格、9 cm 边长。来源：
https://raw.githubusercontent.com/ros-perception/ar_track_alvar/38c599eecd0bf0f4da02164c8401308d12246d82/ar_track_alvar/markers/Markers_0_to_8.png

输出目录为 `~/.ros/aigc_vendor/competition_sim_check`。脚本复制测试路线和任务参数，
单独生成接口配置：模板 ID=113，确认标志仅针对这套测试场景，dry_run=true，模拟
射击话题 `/aigc/competition/dry_run/shoot`。模板接收时效 0.5 秒，适配实测约 5 Hz
的消息流；这仍是接收时间检查，不能替代图像时间戳。

每次整段运行前，在原识别终端 Ctrl+C 后重新运行：

```bash
source /tmp/aigc_vendor_env.sh
"$HOME/.venvs/aigc-vlm/bin/python" "$AIGC_VENDOR_WS/src/abot_vlm/scripts/identify.py"
```

在另一终端检查初始状态：

```bash
source /tmp/aigc_vendor_env.sh
rosparam get /detect
rosservice call /fruit_detection "{}"
```

必须是 255 和 `0,0`。这是为了让主程序获取本次新识别，而非采用前一轮的缓存。
之后只让主程序触发识别，不再手动设置 `/detect=1`。

```bash
roslaunch aigc_navigation competition_mission.launch \
  use_sim_time:=true \
  mission_file:="$HOME/.ros/aigc_vendor/competition_sim_check/mission.yaml" \
  interfaces_file:="$HOME/.ros/aigc_vendor/competition_sim_check/interfaces.yaml" \
  task_file:="$HOME/.ros/aigc_vendor/competition_sim_check/task.yaml"
```

观察实际播报、三个导航目标、对应 113/1/6 目标的对准和终点行为。运行记录默认在
`~/.ros/aigc_competition_check`，应检查 `integration.selection`、各航点结果及
`mission_mode=competition_dry_run`。模拟射击通过不能证明命中。

清理本脚本增添的模型：

```bash
"$HOME/.venvs/aigc-vlm/bin/python" tools/prepare_vendor_sim_check.py --cleanup
```

切换实车时重新核对车载解释器、厂商工作区、在线服务、图像尺寸、真实地图、航点、
模板和 AR 编号、相机/枪口偏差、里程计及 TF、实际发射与停止协议。不能在实车
使用本文的测试世界确认标志或 Gazebo 世界里程计。
