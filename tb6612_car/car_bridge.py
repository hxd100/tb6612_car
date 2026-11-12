#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TB6612 双轮小车的 ROS2 桥接节点。

订阅:  /cmd_vel            geometry_msgs/Twist      线速度 m/s + 角速度 rad/s
发布:  /odom               nav_msgs/Odometry        轮式里程计
       /joint_states       sensor_msgs/JointState   左右轮角度/角速度
       /car/telemetry      std_msgs/Float64MultiArray
                             [左速, 右速, 左目标, 右目标, 左PWM, 右PWM]
TF:    odom -> base_link

差速运动学:
    v_left  = vx - wz * separation / 2
    v_right = vx + wz * separation / 2
再用 ticks_per_meter 把 m/s 换算成下位机的“编码器计数 / 10ms”:
    target = v * ticks_per_meter * 0.01
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster, TransformStamped

from .serial_link import CarLink

BY_ID = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


class CarBridge(Node):

    def __init__(self):
        super().__init__('car_bridge')

        self.declare_parameter('port', BY_ID)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.05)        # m
        self.declare_parameter('wheel_separation', 0.35)    # m
        self.declare_parameter('ticks_per_meter', 6500.0)   # 编码器计数 / 米
        self.declare_parameter('max_linear', 0.40)          # m/s
        self.declare_parameter('max_angular', 1.5)          # rad/s
        self.declare_parameter('max_counts', 40.0)          # counts/10ms 上限
        self.declare_parameter('cmd_timeout', 0.4)          # s，超过则停车
        self.declare_parameter('control_rate', 20.0)        # Hz，下发目标
        self.declare_parameter('odom_rate', 30.0)           # Hz
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        # 镜像安装修正：在 ROS 边界同时翻转该轮的“指令”和“反馈”。
        # 因为两者一起翻，下位机的闭环极性不变（目标/实际始终同号），
        # 而 ROS 看到的符号是正确的。等价于固件里正确实现的 DIRx。
        self.declare_parameter('wheel_invert_left', False)
        self.declare_parameter('wheel_invert_right', False)
        # 镜像安装修正（下位机侧）：启动时发 DIRL/DIRR=1。
        # 固件里 DIRx 已改成「电机 ^ 编码器一起翻」，所以闭环极性和里程计都不受影响。
        self.declare_parameter('dir_invert_left', False)
        self.declare_parameter('dir_invert_right', False)
        # closed_loop: 发 L=/R= 让 STM32 跑 PID；open_loop: 发 TESTL/TESTR 直接给占空比
        self.declare_parameter('control_mode', 'closed_loop')
        self.declare_parameter('open_loop_deadband', 45.0)  # open_loop 模式的起步补偿
        self.declare_parameter('dry_run', False)            # True: 只算不发
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        gp = self.get_parameter
        self.port = gp('port').value
        self.wheel_radius = gp('wheel_radius').value
        self.sep = gp('wheel_separation').value
        self.tpm = float(gp('ticks_per_meter').value)
        self.max_v = gp('max_linear').value
        self.max_w = gp('max_angular').value
        self.max_counts = float(gp('max_counts').value)
        self.cmd_timeout = gp('cmd_timeout').value
        self.invert_l = gp('invert_left').value
        self.invert_r = gp('invert_right').value
        self.mode = gp('control_mode').value
        self.deadband = float(gp('open_loop_deadband').value)
        self.dry_run = gp('dry_run').value
        self.publish_tf = gp('publish_tf').value
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value

        # ---- 串口 ----
        self.link = CarLink(self.port, gp('baud').value)
        if not self.link.ping():
            raise RuntimeError(f'{self.port} 上没有收到 PONG，检查接线/波特率/固件是否 ASCII 版')
        self.link.telemetry(True)

        # 镜像安装修正：在串口边界翻转该轮的指令和反馈
        self.wheel_inv_l = bool(gp('wheel_invert_left').value)
        self.wheel_inv_r = bool(gp('wheel_invert_right').value)
        if self.wheel_inv_l or self.wheel_inv_r:
            self.get_logger().info(
                f'方向修正（ROS 侧同时翻指令+反馈）: '
                f'左={int(self.wheel_inv_l)} 右={int(self.wheel_inv_r)}')

        # 下位机侧镜像修正：掉电不保存，每次启动由本节点重发
        self.dir_inv_l = bool(gp('dir_invert_left').value)
        self.dir_inv_r = bool(gp('dir_invert_right').value)
        if self.dir_inv_l or self.dir_inv_r:
            self.link.send(f'DIRL={1 if self.dir_inv_l else 0}')
            self.link.send(f'DIRR={1 if self.dir_inv_r else 0}')
            self.get_logger().info(
                f'已下发方向修正: DIRL={int(self.dir_inv_l)} DIRR={int(self.dir_inv_r)}')

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._tele = None          # (ls, rs, lt, rt, lo, ro)
        self._x = self._y = self._th = 0.0
        self._v = self._w = 0.0
        self._dist_l = 0.0
        self._dist_r = 0.0
        self._last_frame = None
        self._stop = threading.Event()
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._cmd_stamp = 0.0
        self._dir_l = 0.0          # /wheel_cmd 直接给的左右目标（counts/10ms）
        self._dir_r = 0.0
        self._dir_stamp = 0.0
        self._sent = None
        self._stopped = True

        # ---- ROS 接口 ----
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_tele = self.create_publisher(Float64MultiArray, 'car/telemetry', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        # 左右轮直给（配合 gui.py / 教学示例），单位 counts/10ms
        self.create_subscription(Float64MultiArray, 'wheel_cmd', self.on_wheel_cmd, 10)

        self.create_timer(1.0 / gp('control_rate').value, self.on_control)
        self.create_timer(1.0 / gp('odom_rate').value, self.on_publish)

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            f'car_bridge 启动: {self.port} | 模式={self.mode} | '
            f'轮径={self.wheel_radius} 轮距={self.sep} ticks/m={self.tpm:.0f}'
            + (' | DRY-RUN（不下发运动指令）' if self.dry_run else ''))

    # ------------------------------------------------------------ 串口线程
    def _read_loop(self):
        last_t = None
        while not self._stop.is_set():
            try:
                line = self.link.readline()
            except Exception as exc:                      # 串口断开
                self.get_logger().error(f'串口读取失败: {exc}')
                break
            now = time.monotonic()
            dt = None if last_t is None else now - last_t
            last_t = now
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 6:
                continue
            try:
                tel = tuple(int(p) for p in parts)
            except ValueError:
                continue
            self._integrate(tel, dt)

    def _flip(self, l, r):
        """ROS 帧 <-> MCU 帧 的单轮符号转换（镜像安装修正）。"""
        return (-l if self.wheel_inv_l else l, -r if self.wheel_inv_r else r)

    def _integrate(self, tel, dt):
        """用编码器速率积分位姿。

        下位机 10ms 算一次速度，但遥测 20ms 才发一帧，所以不能假设
        “一帧 = 10ms”，必须按实际接收间隔 dt 积分，否则距离会少算一半。
        遥测里的值单位是 counts/10ms，即 counts/s = 值 × 100。
        """
        if dt is None or dt <= 0.0:
            dt = 0.02
        dt = min(max(dt, 0.001), 0.2)

        dl = self._flip(tel[0], tel[1])[0] * 100.0 * dt / self.tpm   # 左轮米数
        dr = self._flip(tel[0], tel[1])[1] * 100.0 * dt / self.tpm
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / self.sep

        l_spd, r_spd = self._flip(tel[0], tel[1])
        lt, rt = tel[2], tel[3]
        lo, ro = tel[4], tel[5]
        if self.wheel_inv_l:
            lt, lo = -lt, -lo
        if self.wheel_inv_r:
            rt, ro = -rt, -ro

        with self._lock:
            self._dist_l += dl
            self._dist_r += dr
            self._x += ds * math.cos(self._th + dth / 2.0)
            self._y += ds * math.sin(self._th + dth / 2.0)
            self._th = math.atan2(math.sin(self._th + dth), math.cos(self._th + dth))
            self._tele = (l_spd, r_spd, lt, rt, lo, ro)
            self._v = ds / dt
            self._w = dth / dt

    # ------------------------------------------------------------ 控制
    def on_cmd_vel(self, msg: Twist):
        v = max(-self.max_v, min(self.max_v, msg.linear.x))
        w = max(-self.max_w, min(self.max_w, msg.angular.z))
        self._cmd_v, self._cmd_w = v, w
        self._cmd_stamp = self.get_clock().now().nanoseconds / 1e9
        self._stopped = False

    def on_wheel_cmd(self, msg: Float64MultiArray):
        """左右轮直接下目标（counts/10ms），跳过差速换算。"""
        d = list(msg.data)
        if len(d) < 2:
            return
        self._dir_l, self._dir_r = float(d[0]), float(d[1])
        self._dir_stamp = self.get_clock().now().nanoseconds / 1e9
        self._stopped = False

    def _wheel_counts(self, v, w):
        """差速运动学 -> 左右轮 计数/10ms"""
        vl = v - w * self.sep / 2.0
        vr = v + w * self.sep / 2.0
        cl = vl * self.tpm * 0.01
        cr = vr * self.tpm * 0.01
        cl = max(-self.max_counts, min(self.max_counts, cl))
        cr = max(-self.max_counts, min(self.max_counts, cr))
        if self.invert_l:
            cl = -cl
        if self.invert_r:
            cr = -cr
        return cl, cr

    def _open_loop_pwm(self, counts):
        """暂时绕过 PID：把目标计数映射成占空比（含死区补偿）。"""
        if abs(counts) < 0.5:
            return 0
        ratio = min(1.0, abs(counts) / self.max_counts)
        pwm = self.deadband + (100.0 - self.deadband) * ratio
        return int(math.copysign(round(pwm), counts))

    def _targets(self, now):
        """取当前有效的左右目标；两个来源都过期则返回 None。

        来源优先级：谁的时间戳更新就用谁。
          /wheel_cmd  左右轮直接给 counts/10ms（GUI、教学示例用）
          /cmd_vel    差速换算，给导航、teleop_twist_keyboard 用
        """
        fresh_dir = (now - self._dir_stamp) <= self.cmd_timeout
        fresh_cmd = (now - self._cmd_stamp) <= self.cmd_timeout

        if fresh_dir and (not fresh_cmd or self._dir_stamp >= self._cmd_stamp):
            return (max(-self.max_counts, min(self.max_counts, self._dir_l)),
                    max(-self.max_counts, min(self.max_counts, self._dir_r)), 'wheel_cmd')
        if fresh_cmd:
            cl, cr = self._wheel_counts(self._cmd_v, self._cmd_w)
            return cl, cr, 'cmd_vel'
        return None

    def on_control(self):
        now = self.get_clock().now().nanoseconds / 1e9
        tgt = self._targets(now)

        if self.dry_run:
            if tgt is not None and (tgt[0] or tgt[1]):
                self.get_logger().info(
                    f'DRY-RUN[{tgt[2]}]: L={tgt[0]:+.1f} R={tgt[1]:+.1f} counts',
                    throttle_duration_sec=1.0)
            return

        if tgt is None:
            if not self._stopped:                 # 超时只发一次 STOP
                self.link.send('STOP')
                self._stopped = True
                self._sent = None
                self.get_logger().warn(f'指令超时 {self.cmd_timeout}s，已停车',
                                       throttle_duration_sec=2.0)
            return

        cl, cr = tgt[0], tgt[1]
        cl, cr = self._flip(cl, cr)          # ROS 帧 -> MCU 帧
        if self.mode == 'open_loop':
            cmd = f'TESTL={self._open_loop_pwm(cl)},TESTR={self._open_loop_pwm(cr)}'
        else:
            cmd = f'L={int(round(cl))},R={int(round(cr))}'
        if cmd != self._sent:
            self.link.send(cmd)
            self._sent = cmd

    # ------------------------------------------------------------ 发布
    def on_publish(self):
        now = self.get_clock().now()
        with self._lock:
            x, y, th = self._x, self._y, self._th
            v, w = self._v, self._w
            dl, dr = self._dist_l, self._dist_r
            tel = self._tele

        qz = math.sin(th / 2.0)
        qw = math.cos(th / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # 差速底盘只有 vx / wz 有信息量，协方差给经验值
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 1e-3
            odom.twist.covariance[i] = 1e-3
        self.pub_odom.publish(odom)

        if tel is not None:
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [dl / self.wheel_radius, dr / self.wheel_radius]
            js.velocity = [v / self.wheel_radius, w / self.wheel_radius]
            self.pub_js.publish(js)

            msg = Float64MultiArray(data=[float(x) for x in tel])
            self.pub_tele.publish(msg)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)

    # ------------------------------------------------------------ 收尾
    def destroy_node(self):
        self._stop.set()
        if not self.dry_run:
            self.link.send('STOP')
        self.link.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:                     # 串口打不开等致命错误
        print(f'[car_bridge] 启动失败: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()                  # 里面会发 STOP 并关串口
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
