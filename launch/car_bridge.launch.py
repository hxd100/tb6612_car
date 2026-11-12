#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 tb6612_car 桥接节点。

用法:
  ros2 launch tb6612_car car_bridge.launch.py
  ros2 launch tb6612_car car_bridge.launch.py control_mode:=open_loop
  ros2 launch tb6612_car car_bridge.launch.py port:=/dev/ttyUSB0 dry_run:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('tb6612_car'), 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='CH340 串口设备'),
        DeclareLaunchArgument(
            'control_mode', default_value='closed_loop',
            description='closed_loop(L=/R=) 或 open_loop(TESTL/TESTR)'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true 时只计算不下发运动指令'),
        DeclareLaunchArgument(
            'invert_left', default_value='false', description='左轮方向取反'),
        DeclareLaunchArgument(
            'invert_right', default_value='false', description='右轮方向取反（ROS 侧翻目标值）'),
        DeclareLaunchArgument(
            'wheel_invert_left', default_value='false',
            description='左轮镜像修正：同时翻指令和反馈'),
        DeclareLaunchArgument(
            'wheel_invert_right', default_value='false',
            description='右轮镜像修正：同时翻指令和反馈'),
        DeclareLaunchArgument(
            'dir_invert_left', default_value='false',
            description='左轮镜像修正：启动时给下位机发 DIRL=1'),
        DeclareLaunchArgument(
            'dir_invert_right', default_value='true',
            description='右轮镜像修正：启动时给下位机发 DIRR=1'),

        Node(
            package='tb6612_car',
            executable='car_bridge',
            name='car_bridge',
            output='screen',
            parameters=[
                params,
                {'port': LaunchConfiguration('port'),
                 'control_mode': LaunchConfiguration('control_mode'),
                 'dry_run': LaunchConfiguration('dry_run'),
                 'invert_left': LaunchConfiguration('invert_left'),
                 'invert_right': LaunchConfiguration('invert_right'),
                 'wheel_invert_left': LaunchConfiguration('wheel_invert_left'),
                 'wheel_invert_right': LaunchConfiguration('wheel_invert_right'),
                 'dir_invert_left': LaunchConfiguration('dir_invert_left'),
                 'dir_invert_right': LaunchConfiguration('dir_invert_right')},
            ],
        ),
    ])
