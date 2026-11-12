#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口协议封装：只负责收发一行一条 ASCII 指令。"""
import threading
import time

import serial


class CarLink:
    """线程约定：写操作加锁（可能来自 ROS 回调），读只在单个线程里进行。"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self.port = port
        self._write_lock = threading.Lock()
        self.ser = serial.Serial(port, baud, timeout=timeout, write_timeout=1.0)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def send(self, line: str) -> None:
        with self._write_lock:
            self.ser.write(line.strip().encode('ascii') + b'\r\n')
            self.ser.flush()

    def readline(self) -> str:
        return self.ser.readline().decode('ascii', 'replace').strip()

    def telemetry(self, on: bool) -> None:
        self.send(f"T={1 if on else 0}")

    def ping(self, retries: int = 3) -> bool:
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.send("PING")
            deadline = time.time() + 0.5
            while time.time() < deadline:
                if "PONG" in self.readline():
                    return True
        return False

    def close(self) -> None:
        try:
            self.send("STOP")
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
