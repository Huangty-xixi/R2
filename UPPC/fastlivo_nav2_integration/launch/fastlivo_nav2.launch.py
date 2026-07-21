import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 获取包路径 - 使用正确的包名
    pkg_path = FindPackageShare('fastlivo_nav2_integration')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),
        
        # FAST-LIVO2地图转换节点
        Node(
            package='fastlivo_nav2_integration',  # 注意：这里没有下划线
            executable='fast_livo_map_converter',
            name='fast_livo_map_converter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_resolution': 0.05,
                'map_width': 1000,
                'map_height': 1000,
                'origin_x': -25.0,
                'origin_y': -25.0,
                'z_min': -0.1,
                'z_max': 0.5,
                'voxel_size': 0.05,
                'map_update_rate': 2.0,
            }],
            remappings=[
                ('/cloud_registered', '/cloud_registered'),  # FAST-LIVO2点云话题
                ('/map_points', '/map_points'),              # FAST-LIVO2地图点话题
            ]
        ),
        
        # Navigation2 - 如果找不到包，尝试直接路径
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    pkg_path,
                    'config',
                    'nav2_params_fast_livo.yaml'
                ]),
                'autostart': 'true',
            }.items()
        ),
        
        # RViz2配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                pkg_path,
                'config',
                'fast_livo_nav2.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])