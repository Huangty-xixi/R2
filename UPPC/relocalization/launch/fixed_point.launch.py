import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'relocalization'
    pkg_dir = get_package_share_directory(pkg_name)
    nav2_dir = get_package_share_directory('nav2_bringup')

    # --- 参数定义 ---
    # 1. 地图路径
    map_file = LaunchConfiguration('map')
    map_arg = DeclareLaunchArgument('map', default_value='/home/ek/RC2026/src/relocalization/maps/scans.yaml')

    # 2. 参考原点
    ref_x = 0.0
    ref_y = 0.0
    ref_yaw = 0.0

    # 3. 结果保存路径
    save_path = '/home/ek/RC2026/config/startup_offset.yaml'

    # --- 节点定义 ---

    # 1. 静态 TF
    tf_patch_1 = Node(package='tf2_ros', 
                      executable='static_transform_publisher',
                      arguments=['0','0','0','0','0','0', 'odom', 'camera_init'])
    tf_patch_2 = Node(package='tf2_ros', 
                      executable='static_transform_publisher',
                      arguments=['0','0','0','0','0','0', 'aft_mapped', 'base_link'])

    # 2. Pointcloud to LaserScan (3D -> 2D)
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': 0.05, 'max_height': 5.0,
            'angle_min': -3.14159, 'angle_max': 3.14159, 
            'angle_increment': 0.0087,
            'qos_reliability': 'reliable',         # 强制可靠传输
            'qos_durability': 'transient_local',   # 配合 AMCL
            'range_max': 30.0, 
            'use_inf': True
        }]
    )

    # 3. Nav2 纯定位模式
    nav2_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params_fixed_point.yaml'),
            'autostart': 'true'
        }.items()
    )

    # 4. 偏差记录节点
    pose_logger = Node(
        package=pkg_name,
        executable='pose_logger',
        name='pose_logger',
        output='screen',
        parameters=[{
            'ref_x': ref_x,
            'ref_y': ref_y,
            'ref_yaw': ref_yaw,
            'save_path': save_path,
            'one_shot': True  # 只记录一次
        }]
    )

    # 5. Rviz2
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')]
    )

    return LaunchDescription([
        map_arg,
        tf_patch_1, tf_patch_2,
        pcl_to_scan,
        nav2_loc,
        pose_logger,
        rviz
    ])