#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实车一键启动：STM32 底盘 + 机器人模型 + 真实雷达 + 深度相机。

按需组合，用参数关掉暂时不用的部分：

    # 全套
    ros2 launch tb6612_car real_bringup.launch.py

    # 底盘没上电，只跑雷达和相机
    ros2 launch tb6612_car real_bringup.launch.py use_chassis:=false

    # 只跑底盘
    ros2 launch tb6612_car real_bringup.launch.py use_lidar:=false use_camera:=false

    # 再加 RViz
    ros2 launch tb6612_car real_bringup.launch.py rviz:=true

各节点作用：
  car_bridge    /cmd_vel -> 串口 -> STM32；发布 /odom /joint_states /car/telemetry
                TF: odom -> base_footprint
  rsp           my_bot 的 robot_description，TF: base_footprint -> base_link -> 各部件
  lidar_node    /dev/lidar -> /scan（frame_id: laser_frame）
  astra_camera  /camera/color|depth|ir/image_raw（Orbbec Astra Pro Plus）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb = get_package_share_directory('tb6612_car')
    mybot = get_package_share_directory('my_bot')
    lidar = get_package_share_directory('lidar_pkg')
    astra = get_package_share_directory('astra_camera')

    port = LaunchConfiguration('port')
    dir_inv_r = LaunchConfiguration('dir_invert_right')
    rviz = LaunchConfiguration('rviz')
    use_chassis = LaunchConfiguration('use_chassis')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mybot, 'launch', 'rsp.launch.py')),
    )

    bridge = Node(
        package='tb6612_car',
        executable='car_bridge',
        name='car_bridge',
        output='screen',
        condition=IfCondition(use_chassis),
        parameters=[
            os.path.join(tb, 'config', 'params.yaml'),
            {
                'port': port,
                # 实车有 base_footprint：odom 挂到它上面，slam_toolbox / nav2 才能用
                'base_frame': 'base_footprint',
                # 右轮镜像：下位机 DIRR=1
                'dir_invert_right': dir_inv_r,
            },
        ],
    )

    lidar_node = Node(
        package='lidar_pkg',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        condition=IfCondition(use_lidar),
        parameters=[os.path.join(lidar, 'config', 'lidar_params.yaml')],
    )

    camera = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(astra, 'launch', 'astro_pro_plus.launch.xml')),
        condition=IfCondition(use_camera),
        launch_arguments={'depth_align_color': 'true'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('dir_invert_right', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'use_chassis', default_value='true',
            description='底盘桥接（底盘没上电时设 false）'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='激光雷达驱动'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Astra 深度相机驱动'),

        bridge,
        rsp,
        lidar_node,
        camera,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(tb, 'config', 'car.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
