#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_dir = os.path.expanduser('~/qr_ws/dyn_qr')

    return LaunchDescription([
        DeclareLaunchArgument(
            'images_dir',
            default_value=default_dir,
            description='Directory containing current.png'
        ),
        DeclareLaunchArgument(
            'default_qr',
            default_value='current.png',
            description='Default image file name in images_dir'
        ),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='5.0',
            description='Publish rate of fake camera image'
        ),

        # 1) image_publisher -> /camera/image_raw
        Node(
            package='qr_code_detection',
            executable='image_publisher',
            name='image_publisher',
            output='screen',
            parameters=[{
                'images_dir': LaunchConfiguration('images_dir'),
                'default_qr': LaunchConfiguration('default_qr'),
                'topic': '/camera/image_raw',
                'publish_hz': LaunchConfiguration('publish_hz'),
                'select_topic': '/qr_select',
            }]
        ),

        # 2) qr_decoder（窗口显示 + 发布 /qrcode/text）
        Node(
            package='qr_code_detection',
            executable='qr_decoder',
            name='qr_decoder',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'output_topic': '/qrcode/text',
                'show_window': True,
                'resize_scale': 2.0,
                'use_threshold': True,
                'window_name': 'QR Decoder',
                'window_width': 640,
                'window_height': 480,
            }]
        ),

        # 3) qrcode_control（扳机一次执行 + 支持序列 + 发布自定义信息）
        Node(
            package='qr_code_control',
            executable='qrcode_control',
            name='qrcode_control',
            output='screen',
            parameters=[{
                'input_topic': '/qrcode/text',
                'fire_topic': '/qr_fire',
                'cmd_vel_topic': '/cmd_vel',           # 3D机器人话题
                'info_topic': '/qr/custom_info',
                'linear_speed': 0.2,
                'angular_speed': 1.0,
            }]
        ),
    ])

