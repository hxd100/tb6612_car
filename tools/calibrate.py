#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定 ticks_per_meter：把车推着走一段已知距离，统计编码器累计计数。

用法:
    python3 calibrate.py 10 1.0
        参数1: 采样时长（秒）
        参数2: 这段时间里车实际走的距离（米）

做法：先把车放好，运行后立刻用手推着走 1 米（或让它自己跑），
时间到了它会打印累计计数和推算出的 ticks_per_meter。
"""
import sys
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


def main() -> int:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    dist = float(sys.argv[2]) if len(sys.argv) > 2 else None

    s = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(0.2)
    s.write(b"T=1\r\n")
    s.flush()

    total_l = total_r = 0.0
    last_t = None
    t0 = time.time()
    print(f"开始采样 {secs:.1f}s，现在推车……")
    while time.time() - t0 < secs:
        line = s.readline()
        now = time.monotonic()
        dt = None if last_t is None else now - last_t
        last_t = now
        if not line:
            continue
        p = line.decode('ascii', 'replace').strip().split(',')
        if len(p) != 6:
            continue
        try:
            v = [int(x) for x in p]
        except ValueError:
            continue
        if dt is None or dt <= 0:
            continue
        dt = min(max(dt, 0.001), 0.2)
        # 单位是 counts/10ms，换算成这段时间内的计数
        total_l += v[0] * 100.0 * dt
        total_r += v[1] * 100.0 * dt

    s.write(b"STOP\r\n")
    s.close()

    avg = (total_l + total_r) / 2.0
    print(f"\n累计计数: 左 {total_l:.0f}  右 {total_r:.0f}  平均 {avg:.0f}")
    if dist and dist > 0:
        print(f"行走距离: {dist:.3f} m")
        print(f"=> ticks_per_meter ≈ {avg / dist:.0f}")
        print("   把这个值填到 config/params.yaml 的 ticks_per_meter")
    else:
        print("（没给距离参数，只打印了计数）")


if __name__ == '__main__':
    sys.exit(main())
