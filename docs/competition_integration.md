> **最新实测（2026-10-22）**：运行 `aigc_20261022T085545_bc104676`，82.23秒，COMPLETED/0。官方识别返回数学1、榴莲6；模板113、AR1、AR6及终点均一次成功。三个任务完成对准和模拟射击，object_kp=0.002。起终点语音消息均已发布，实际听音待确认。本次覆盖一个样例和测试地图；真实发射、命中及正式场地尚待验证。下文“尚未联调”等早期状态以本条更新为准。

# 赛方接口适配：ROS1 Melodic，首轮使用模拟射击

本轮依据用户提供的《无人车任务挑战赛(2).pdf》第 9、10 页及两份源码，
补齐“触发任务识别→读取数学/水果结果→播报→模板任务→数学 AR 任务→
水果 AR 任务→终点播报”。导航仍通过 move_base Action 调用已有导航栈。
不复制、修改赛方识别算法、云服务客户端或硬件驱动，不复制任何 API 密钥。

这是**代码适配与本地替身测试完成**的版本。尚未在用户 Ubuntu 上完成这些
赛方接口的 ROS 联调，不代表比赛地图、真实识别率、发射或命中已经通过。
旧 `task_mission_sim.launch` 仍运行原来的 ID 0/1/2 测试场景。

## 1. 已核对的接口与源码差异

| 接口 | 源码依据 | 本轮调用方式 |
| --- | --- | --- |
| `/detect` | `abot——01/abot/src/abot_vlm/scripts/identify.py` 读取参数 | **参数**置 1；不是发布同名话题 |
| `/fruit_detection` | 上述 `identify.py` | `std_srvs/Trigger`，严格解析 `"math,fruit"`；数学 1～5、水果 6～8；`0,0` 不算成功 |
| `/object_position` | `abot_find/src/ros/print_objects_detected_node.cpp` | `geometry_msgs/Point`：x/y 为图像像素中心、**z 为模板 ID**；255 表示无目标 |
| `/ar_pose_marker` | `ar_track_alvar` | 按数学答案、水果编码选择 ID，继续使用每个 marker 的自身图像 header |
| `/voiceWords` | `robot_voice/src/tts_subscribe.cpp` | `std_msgs/String`；等待订阅者后发布，记录 `published`，不冒充声音播放确认 |
| `/tts_service`（可选） | `TTS_audio/scripts/TTS.py`、`srv/StringService.srv` | `TTS_audio/StringService`，请求 `data`、响应 `result`；检查成功文本 |
| 导航 | 原巡航示例与 ROS move_base | 使用 Action 的真实结果、有限重试和超时取消 |
| 射击 | `shoot_cmd/src/shoot_control.cpp` | 命令词沿用 `shoot` / `stopshoot`，本轮**仅发往独立模拟话题** |

注意：`2026_demo/abot_ws/src/abot_vlm/scripts/identify_service.py` 实际返回的是
**单个数字字符串**，与任务书的双结果接口不相同。不能因为服务名相同就把它
当成双结果版本。当前适配器遇到这种结果会报错并停止，不发送导航目标。

赛方源码还有直接发布 `/cmd_vel` 和射击指令的旧示例。运行新主程序时，不要
同时启动旧 `navigation_multi_goals*`、`track_tag/ar_track`、`find_shoot_demo`
或其他对准控制器。保留检测节点，底盘速度只由导航阶段或当前任务阶段控制。

## 2. 识别缓存与时间限制

双结果版本的服务只读取缓存，没有请求编号或图像时间戳；`/detect` 在保存图片后、
云端推理完成前就重置为 255。因此不能把“参数已重置”当成推理完成。

每次完整任务应新启动一次该 `identify.py`，初始缓存必须为 `0,0`。适配器先读取
空缓存，再设置 `/detect=1`，轮询直到得到有效新结果；已有非空缓存、错误格式、
异常或超时都会停止。它不会为了绕过旧缓存而改写赛方源码。识别总等待为
120 秒、单次服务等待为 5 秒，可在配置里调整，均按墙上时间计算。

超时的 RPC 可能仍在服务端执行，迟到结果会被丢弃，任务停止，不再重复调用。
运行时应只有一个识别触发者；原接口没有跨客户端请求隔离能力。

## 3. 模板与语音的可验证边界

`/object_position` 没有 header，只能检查**接收时间**、坐标范围、模板 ID、
连续新接收样本及车身停稳；无法从该消息证明图像采集时间。记录中明确保存
`detection_freshness=receive_time_only_no_image_header`。AR 路径仍保留图像时间检查。
模板对准默认中心 x=320、误差容许 3 像素；速度限幅与任务总期限沿用现有模块。
图像尺寸、中心和模板 ID 需要按实际相机/模板配置。

语音默认采用源码中存在的 `/voiceWords` 话题接口。若使用 `TTS_audio`，将
`voice_backend` 改成 `service`，并先 source 能找到 `TTS_audio/StringService`
生成类型的工作区。服务的正常返回只能证明它报告处理完成，仍需现场听音确认。
初始播报失败时不开始航点；终点播报失败时整场不标为 COMPLETED。

射击后端仍为 `SimShooter`，话题强制改成 `/aigc/competition/dry_run/shoot`，
不会向默认物理 `/shoot` 发送脉冲。运行记录为 `competition_dry_run`，每次反馈
仍标注 `backend=simulated`、`hit=null`。配置 `dry_run=false` 会被拒绝；原串口代码
没有可直接采用的发射完成反馈约定，本轮没有虚构这类确认或命中结果。

## 4. Ubuntu 编译与只读检查

在已获取本分支后：

```bash
source /opt/ros/melodic/setup.bash
cd ~/Desktop/workplace/AIGC/catkin_ws
catkin_make -j2
source devel/setup.bash

# 只查看 ROS 图，不触发云端识别、不发送速度或射击指令。
# 赛方导航、传感器、识别、模板检测与语音节点应已分别运行。
rosrun aigc_navigation check_vendor_interfaces.py

# 如果启动的是 TTS_audio 服务端，使用这一项：
rosrun aigc_navigation check_vendor_interfaces.py _voice_backend:=service
```

应先 source **选定的一套**赛方工作区，再 source AIGC 工作区。两份源码有同名包，
不能简单叠加后假定使用了期望的版本。可用 `rospack find abot_vlm`、
`rospack find find_object_2d`、`rospack find robot_slam` 检查实际解析路径。
本项目的 CMake 不编译赛方包，赛方包的依赖与运行环境仍由其原工作区提供。

只读工具核对话题/服务类型和语音订阅者。全部 OK 也仅代表接口已发现，
不证明识别内容、图像新鲜度、导航精度或语音效果正确。

## 5. 配置与实际联调入口

`config/competition_mission.example.yaml` 仍是旧测试地图的坐标，
`config/competition_interfaces.yaml` 的模板 ID=1 也是示例。模板→数学→水果的
任务顺序按第 10 页流程图实现；数学/水果结果直接用作 AR ID，需核对实际靶标编号。
完整的隔板布局、绕行航点、起终点区域容差、任务板视角尚需赛场标定，不能使用
这套示例坐标宣布正式比赛验收。

请复制为自己的配置，再按当前地图和训练模板填写。核对后将接口配置中的
`profile_confirmed` 设置为 true。它表示参数来源已核对，不表示测试已经通过。
默认 false 会在任务开始前明确报配置错误，避免示例坐标被误用。

```bash
cp "$(rospack find aigc_navigation)/config/competition_mission.example.yaml" /tmp/aigc_competition_mission.yaml
cp "$(rospack find aigc_navigation)/config/competition_interfaces.yaml" /tmp/aigc_competition_interfaces.yaml

# 编辑这两个文件，填写实际航点/模板 ID/语音后端并核对参数。
# 外部赛方节点就绪后，单独启动我们的主程序：
roslaunch aigc_navigation competition_mission.launch \
  mission_file:=/tmp/aigc_competition_mission.yaml \
  interfaces_file:=/tmp/aigc_competition_interfaces.yaml
```

若接在 Gazebo 上，额外传入 `use_sim_time:=true`。本入口不会另起 Gazebo、
相机、雷达、导航或赛方检测节点，避免同名节点与驱动争用。
不要把旧测试世界中的 AR0 图案当成已经训练完成的正式模板目标。

本轮通过 ROS1 Melodic 源码接口适配；任务材料中的 ROS2 示意不等于已完成 ROS2
移植，也不作为比赛对软件版本限制的确认。

## 6. 验证结果与待做项

本地执行：

```bash
python3 tools/regression_test.py
python3 tools/competition_regression_test.py
```

原有 84 项检查通过；新增 21 项测试通过，包含完整任务的动态 ID 顺序、
识别失败零导航、旧缓存拒绝、阻塞 RPC 墙上超时、模板 ID/像素/时效过滤、
重复样本不累计、最终对准复核、语音服务响应检查、终点播报失败不记完成、
独立模拟射击话题与 JSON 记录。另检查 Python 2 语法、XML/YAML 配置与补丁空白。
这些是本地替身测试，不是 Ubuntu ROS、云识别、真实检测或实车验证。

下一轮 Ubuntu 联调需要记录：实际 source 路径、接口检查输出、双结果识别输出、
三个任务分支的导航/对准结果、现场播报、完整 JSON 与录屏。
真实发射后端、完成反馈和命中判定按实际设备接口另行接入。
