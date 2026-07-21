import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')

    kfs_config_path = os.path.join(pkg_rc2026_field, 'config', 'kfs_config.yaml')
    world_path = os.path.join(pkg_rc2026_field, 'worlds', 'robocon2026.world')

    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_rc2026_field, 'models')
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_path,
            '-s',
            'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    kfs_manager = Node(
        package='rc2026_field',
        executable='kfs_manager',
        name='kfs_manager',
        output='screen',
        parameters=[{'use_sim_time': True, 'config_path': kfs_config_path}]
    )

    field_gui = Node(
        package='rc2026_field',
        executable='field_gui',
        name='field_gui',
        output='screen',
        parameters=[{'use_sim_time': True, 'config_path': kfs_config_path}]
    )

    return LaunchDescription([
        set_model_path,
        gazebo_server,
        gazebo_client,
        kfs_manager,
        field_gui
    ])
