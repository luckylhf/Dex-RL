"""Launch the MuJoCo EVT2 simulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model', default_value='dex_v3',
        description='Robot model name registered in simulator_view_asyn.py')
    config_arg = DeclareLaunchArgument(
        'robot_config', default_value='full',
        description='V4 ROS policy interface requires full (29 motor channels)')

    mujoco_node = Node(
        package='mujoco_node',
        executable='mujoco_node',
        name='mujoco_simulator_dex',
        output='screen',
        arguments=['-m', LaunchConfiguration('model'),
                   '-c', LaunchConfiguration('robot_config')],
    )

    return LaunchDescription([model_arg, config_arg, mujoco_node])
