#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实车一键启动：STM32 底盘 + 机器人模型 + 真实雷达。

    ros2 launch tb6612_car real_bringup.launch.py
    ros2 launch tb6612_car real_bringup.launch.py rviz:=true
    ros2 launch tb6612_car real_bringup.launch.py dir_invert_right:=false

启动内容：
  car_bridge   /cmd_vel -> 串口 -> STM32；发布 /odom /joint_states /car/telemetry
               TF: odom -> base_footprint
  rsp          my_bot 的 robot_description，TF: base_footprint -> base_link -> 各部件
  lidar_node   /dev/lidar -> /scan（frame_id: laser_frame）
  rviz2        可选
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb = get_package_share_directory('tb6612_car')
    mybot = get_package_share_directory('my_bot')
    lidar = get_package_share_directory('lidar_pkg')

    port = LaunchConfiguration('port')
    dir_inv_r = LaunchConfiguration('dir_invert_right')
    rviz = LaunchConfiguration('rviz')
    use_lidar = LaunchConfiguration('use_lidar')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mybot, 'launch', 'rsp.launch.py')),
    )

    bridge = Node(
        package='tb6612_car',
        executable='car_bridge',
        name='car_bridge',
        output='screen',
        parameters=[
            os.path.join(tb, 'config', 'params.yaml'),
            {
                'port': port,
                # 实车有 base_footprint：odom 挂到它上面，slam_toolbox / nav2 才能用
                'base_frame': 'base_footprint',
                # 本车右轮镜像，下位机 DIRR=1
                'dir_invert_right': dir_inv_r,
            },
        ],
    )

    lidar_node = Node(
        package='lidar_pkg',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[os.path.join(lidar, 'config', 'lidar_params.yaml')],
        condition=IfCondition(use_lidar),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port',
                              default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('dir_invert_right', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='关闭雷达驱动（只跑底盘时设 false）'),

        bridge,
        rsp,
        lidar_node,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(tb, 'config', 'car.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
