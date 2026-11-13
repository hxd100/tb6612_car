# tb6612_car — ROS2 桥接包

把 STM32（TB6612 双轮小车，串口 ASCII 协议）接进 ROS2 Humble。

```
/cmd_vel ──> car_bridge ──串口──> STM32 (左右轮独立 PID) ──> TB6612 ──> 电机
             │
             └──> /odom, /joint_states, /car/telemetry, TF(odom->base_link)
```

闭环（编码器 PID）仍然在 STM32 上跑，ROS2 只给"目标速度"、读"实际速度"。
好处是 ROS 节点卡顿/重启不会让车失控，而且 10ms 控制周期不受 Jetson 负载影响。

## 1. 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select tb6612_car --symlink-install
source install/setup.bash
```

## 2. 运行

```bash
# 正常模式（走 STM32 的 PID）
ros2 launch tb6612_car car_bridge.launch.py

# 只算不发，安全验证
ros2 launch tb6612_car car_bridge.launch.py dry_run:=true

# 极性没修好时，临时用开环直接给占空比
ros2 launch tb6612_car car_bridge.launch.py control_mode:=open_loop
```

键盘遥控（另一个终端）：

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> `teleop_twist_keyboard` 默认发 `/cmd_vel`，直接就能用。注意它默认速度是 0.5 m/s，
> 超过本车的 `max_linear: 0.40` 会被限幅；想慢一点启动前先把速度调低。

左右轮滑条 GUI（需要桌面环境，SSH 里没有 `DISPLAY` 跑不了）：

```bash
ros2 run tb6612_car wheel_gui
```

两个滑条分别控制左右轮（±40 counts/10ms），实时显示实际速度/目标/PWM，
关窗口时会自动发 0 停车。它发的是 `/wheel_cmd`（`Float64MultiArray [左, 右]`），
和 `/cmd_vel` 两个入口按时间戳取新者生效。

## 3. 话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 线速度 m/s + 角速度 rad/s |
| 发布 | `/odom` | `nav_msgs/Odometry` | 轮式里程计（含 twist） |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | `left_wheel_joint` / `right_wheel_joint` |
| 发布 | `/car/telemetry` | `std_msgs/Float64MultiArray` | 左速,右速,左目标,右目标,左PWM,右PWM |
| TF | `odom -> base_link` | | 位姿 |

参数在 `config/params.yaml`，也可以命令行覆盖：

```bash
ros2 run tb6612_car car_bridge --ros-args -p max_linear:=0.2 -p ticks_per_meter:=6300.0
```

## 4. 标定 ticks_per_meter

`ticks_per_meter` = 轮子走 1 米累计的编码器计数，决定里程计准不准。出厂默认 6500 只是估计值。

```bash
python3 tools/calibrate.py 10 1.0     # 采样 10 秒，期间把车推 1 米
```

得到结果后写回 `config/params.yaml`。也可以用车自己跑：先 `wheel -l 25 -r 25 -s 3`，
量出实际位移再反算。

## 5. 和 my-bot / RViz 一起看

一条命令把桥接、机器人模型、RViz 全起来：

```bash
ros2 launch tb6612_car bringup.launch.py rviz:=true
```

等价的手动方式：

```bash
ros2 launch my_bot rsp.launch.py     # 机器人模型，订阅 /joint_states 发 TF
ros2 run tf2_tools view_frames       # 生成 TF 树 PDF
rviz2 -d ~/ros2_ws/install/tb6612_car/share/tb6612_car/config/car.rviz
```

`car.rviz` 已经配好：Fixed Frame = `odom`，含 Grid / RobotModel / TF / Odometry。

已验证的 TF 链：

```
odom ──(car_bridge)──> base_link ──(robot_state_publisher)──> left_wheel / right_wheel / ...
                        └─ base_link -> left_wheel: y=+0.175, rpy=-90°
```

本包的 `wheel_separation: 0.35` 和 `wheel_radius: 0.05` 与
`my-bot/description/robot_core.xacro` 一致（关节 `origin y=±0.175`）。

## 6. 安全设计

- `cmd_timeout: 0.4`：超过 0.4 秒没收到 `/cmd_vel` 就发 `STOP`，ROS 节点崩了车会停。
  （注意下位机自己的 `CHASSIS_CMD_TIMEOUT_MS` 默认仍是 0，建议也改成 500~1000，双保险。）
- 进程退出时（Ctrl+C）会发 `STOP` 并关串口。
- `max_linear` / `max_angular` / `max_counts` 三重限幅。
- `dry_run:=true` 时完全不发运动指令，只跑算法和话题，适合先验证接线。

## 8. 实测记录（2026-11-10，本车）

### 极性修复后（刷 `CHASSIS_ENCODER_INVERT=1` 固件）

| 测试 | 结果 |
| --- | --- |
| 开环极性 | `TESTL=+100` → **+45**，`-100` → -46；`TESTR=+100` → **+40**，`-100` → -40 ✅ |
| 闭环 `L=25,R=25` | 实际 **+25 / +25**，PWM 57 / 64 ✅ |
| 右轮镜像 | 两轮同 +20 时**原地打转** → 节点启动发 `DIRR=1` 后**直走**（yaw 由 2° 降到 1°）✅ |
| 闭环驱动 0.2 m/s × 1.2 s | dx=**+0.262 m**，yaw=**-0.3°**，轮速 13/14 |
| 标定行程 0.15 m/s × 3.0 s | dx=**+0.449 m**（理论 0.45），yaw=-0.5°，轮速 11/11 |

对比修复前的开环：dx=-0.409 m、yaw=-14.5°（全速倒转还画龙）。
现在方向正确、走直线、里程计自洽。

### 修复前（供对照）

| 测试 | 结果 |
| --- | --- |
| 话题 / TF / joint_states | 全部正常 |
| 运动学换算（dry-run） | `v=0.1` → L=+6.5 R=+6.5；`v=0.1,w=0.5` → L=+0.8 R=+12.2；`v=0,w=1` → L=-11.4 R=+11.4，与理论一致 |
| 开环驱动 1.2s（无 invert） | dx=**-0.409 m**，yaw=-14.5°，轮速 19/22 |
| 开环驱动 1.2s（invert 双侧） | dx=**+0.360 m**，yaw=+20.3° |
| cmd_vel 看门狗 | 停发后 0.4s 自动 `STOP` |
| Ctrl+C 退出 | 无异常堆栈，发 STOP，串口释放 |

### 一个容易踩的测量坑

下位机 50Hz 发遥测，如果脚本里先 `sleep` 再采样，**缓冲里堆的是几秒前的旧帧**，
读出来的全是历史数据。任何"发指令 → sleep → 采样"的测试，采样前必须
`serial.reset_input_buffer()`，否则会得出"电机没转"或"符号乱跳"的错误结论。

## 9. 常见问题

| 现象 | 处理 |
| --- | --- |
| 启动报 "没有收到 PONG" | 串口不对/被占用；`ls -l /dev/serial/by-id/`；关掉 `carterm.sh` |
| `/odom` 距离明显偏小/偏大 | `ticks_per_meter` 没标定 |
| 车反向、原地打转 | `invert_left` / `invert_right`，或修 `DIRL/DIRR` |
| 一发 cmd_vel 就全速反转 | 反馈极性还没修（见 `~/car/README.md` 第 8 节），先用 `control_mode:=open_loop` 应急 |
| 转向方向反 | 把 `wheel_separation` 的符号理解为"左正右负"，或在节点里对调左右目标 |

## 10. 调试记录（2026-11-13）

### 10.1 右轮镜像：`DIRR=1` 必须保留

右电机是镜像安装的，`+目标` 会让它物理上往后转。曾在重接线后试过取消修正
（`dir_invert_right:=false`），结果车辆无法走直线，已恢复为 `true`。

| 配置 | 结果 |
| --- | --- |
| `dir_invert_right: true` | 两轮同向，可直行 |
| `dir_invert_right: false` | 一前一后，原地打转 |

> 注意：**编码器读数看不出来**。两轮编码器都会报正数，`/odom` 也显示在直行，
> 只有肉眼能看到车在打转。判断方向必须看轮子，不能只看遥测。

### 10.2 左轮 PWM 打到 100 却不转

现象：`L=16` 时左轮目标 16、PWM 饱和到 100，实际速度始终 0；右轮正常。

原因：电机线/编码器线接触不良，重新插拔后恢复。

排查顺序：

1. `python3 car_ctl.py test -l 100 -s 2`（开环，绕过 PID 和编码器）
   - 轮子不转 → 查电机线、TB6612 的 A 通道
   - 轮子转但遥测是 0 → 查编码器线（PA0/PA1）
2. 断电手转轮子：转起来很轻 = 电机线断了；有明显阻力 = 电机是通的

### 10.3 串口"能打开但没数据"的排查

现象：端口能打开、能写入，但一个字节都收不到。

| 检查 | 命令 | 判据 |
| --- | --- | --- |
| 设备在不在 | `ls -l /dev/serial/by-id/` | 有 `usb-1a86_...` |
| 波特率对不对 | 逐个试 9600 / 57600 / 115200 / 230400 | 只有 115200 出干净数据 |
| USB 有没有反复掉线 | `dmesg \| grep ch341` | 没有 disconnect/attach 循环 |
| CH340 本身坏没坏 | 短接模块的 TX 和 RX，再发 `PING` | 能收到自己发的字节 = 模块完好 |

本次故障的真实原因是 **STM32 侧没有供电**：CH340 由 Jetson 的 USB 供电，所以
USB 一直枚举正常、端口也能打开，但 MCU 没电就不会回任何数据。

### 10.4 供电与共地

当前方案：`7.4V 2S 锂电 → TB6612 的 VM`，Jetson 由独立电源供电。

三条规则：

1. **电池负极先接 TB6612 的 GND**（粗线 ≥0.5mm²，尽量短）
2. **STM32 的 GND 从同一个星点引出**（细线即可，只走信号电流）
3. **不要让电机电流走 USB 地线**：USB 地只有 28AWG，几安培会烧 CH340

验收方法：电机满速时，用万用表测 STM32 GND 与 TB6612 GND 之间的电压差，
应 **< 50 mV**。超过 200 mV 说明地线太细或接触不良。

### 10.5 键盘操控速查

```bash
# 终端 1：启动底盘（没有雷达时加 use_lidar:=false）
ros2 launch tb6612_car real_bringup.launch.py use_lidar:=false

# 终端 2：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` 前进、`j`/`l` 转向、`k` 停、`q`/`z` 调速。默认 0.5 m/s 对这台车太快，
先按几次 `z` 降到 0.2。

**松开按键不会停**，必须按 `k`。
