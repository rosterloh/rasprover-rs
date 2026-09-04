import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Use simulation (Gazebo) if true'
    )

    package_name = 'rasprover_gz'
    description_pkg_share = get_package_share_directory('rasprover_description')

    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    world_file_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'empty.sdf')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file_path]}.items()
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'rasprover', '-topic', 'robot_description'],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output='screen'
    )

    return LaunchDescription([
        sim_mode_arg,
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[os.path.join(description_pkg_share, '..')]),
        gazebo_launch,
        spawn_model,
        gz_bridge,
    ])
