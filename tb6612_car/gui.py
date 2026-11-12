#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
左右轮滑条遥控 GUI（tkinter）。

发布: /wheel_cmd   std_msgs/Float64MultiArray  [左目标, 右目标]  单位 counts/10ms
订阅: /car/telemetry                            [左速,右速,左目标,右目标,左PWM,右PWM]

用法:
    ros2 run tb6612_car wheel_gui
    （需要桌面环境，SSH 里没有 DISPLAY 跑不了）

注意：先启动桥接节点
    ros2 launch tb6612_car car_bridge.launch.py
"""
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

MAX_SPEED = 40          # counts/10ms，满速约 45
PUBLISH_MS = 100        # 10Hz 持续下发，配合桥接的 0.4s 看门狗


class WheelGui(Node):

    def __init__(self):
        super().__init__('wheel_gui')
        self.pub = self.create_publisher(Float64MultiArray, 'wheel_cmd', 10)
        self.create_subscription(Float64MultiArray, 'car/telemetry', self.on_tele, 10)
        self.tele = None

    def on_tele(self, msg):
        self.tele = list(msg.data)

    def send(self, left, right):
        self.pub.publish(Float64MultiArray(data=[float(left), float(right)]))


class App:

    def __init__(self, root, node):
        self.root = root
        self.node = node
        root.title('TB6612 小车遥控')
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.vl = tk.IntVar(value=0)
        self.vr = tk.IntVar(value=0)

        frm = ttk.Frame(root, padding=12)
        frm.grid(row=0, column=0, sticky='nsew')

        ttk.Label(frm, text='左轮', font=('', 11, 'bold')).grid(row=0, column=0, sticky='w')
        ttk.Scale(frm, from_=-MAX_SPEED, to=MAX_SPEED, orient='horizontal',
                  variable=self.vl, length=320).grid(row=1, column=0, columnspan=2, sticky='we')
        self.lbl_l = ttk.Label(frm, text='0', width=6)
        self.lbl_l.grid(row=1, column=2)

        ttk.Label(frm, text='右轮', font=('', 11, 'bold')).grid(row=2, column=0, sticky='w', pady=(10, 0))
        ttk.Scale(frm, from_=-MAX_SPEED, to=MAX_SPEED, orient='horizontal',
                  variable=self.vr, length=320).grid(row=3, column=0, columnspan=2, sticky='we')
        self.lbl_r = ttk.Label(frm, text='0', width=6)
        self.lbl_r.grid(row=3, column=2)

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=3, pady=12, sticky='we')
        ttk.Button(btns, text='停止 (STOP)', command=self.stop).pack(side='left', expand=True, fill='x')
        ttk.Button(btns, text='左边/右边 归零', command=self.zero).pack(side='left', expand=True, fill='x')

        self.status = ttk.Label(frm, text='等待遥测…', font=('Courier', 10), justify='left')
        self.status.grid(row=5, column=0, columnspan=3, sticky='w')

        ttk.Label(frm, text=f'范围 ±{MAX_SPEED} counts/10ms（满速约 45）',
                  foreground='#666').grid(row=6, column=0, columnspan=3, sticky='w', pady=(8, 0))

        self.tick()

    def zero(self):
        self.vl.set(0)
        self.vr.set(0)

    def stop(self):
        self.zero()
        self.node.send(0, 0)

    def tick(self):
        l, r = int(self.vl.get()), int(self.vr.get())
        self.lbl_l.config(text=f'{l:+d}')
        self.lbl_r.config(text=f'{r:+d}')
        self.node.send(l, r)

        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.node.tele
        if t and len(t) == 6:
            self.status.config(
                text=f'实际  左 {t[0]:+4.0f}  右 {t[1]:+4.0f}\n'
                     f'目标  左 {t[2]:+4.0f}  右 {t[3]:+4.0f}\n'
                     f'PWM   左 {t[4]:+4.0f}  右 {t[5]:+4.0f}')
        self.root.after(PUBLISH_MS, self.tick)

    def on_close(self):
        try:
            self.node.send(0, 0)
        except Exception:
            pass
        self.root.quit()


def main():
    rclpy.init()
    node = WheelGui()
    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    finally:
        node.send(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
