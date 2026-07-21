import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    this_package_name='kfs_seg'

    realsense_dir = get_package_share_directory('realsense2_camera')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true'
        }.items()
    )

    # 3. YOLO 节点
    yolo_node = Node(
        package=this_package_name,
        executable='yolov8_seg',
        name='yolov8_seg',
        output='screen',
        parameters=[
            {'device': 'cuda:0'},
        ],
    )

    return LaunchDescription([
        realsense_launch,
        yolo_node
    ])