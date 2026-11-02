"""Display the original HARTECH arm with an optional parallel gripper."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def setup(context):
    package = Path(get_package_share_directory('robot_description'))
    addon = Path(get_package_share_directory('hartech_tools'))
    model = addon / 'urdf/robot_with_tool.urdf.xacro'
    tool = LaunchConfiguration('tool').perform(context)
    description = ParameterValue(
        xacro.process_file(str(model), mappings={'tool': tool}).toxml(), value_type=str)
    config = yaml.safe_load((package / 'config/robot_parameters.yaml').read_text(encoding='utf-8'))
    pose = LaunchConfiguration('pose').perform(context)
    if pose not in ('home', 'zero'):
        raise ValueError('pose must be home or zero')
    zeros = {f'zeros.{joint}': values['home'] if pose == 'home' else 0.0
             for joint, values in config['joints'].items()}
    if tool == 'parallel':
        zeros['zeros.hartech_left_finger_joint'] = 0.015
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'robot_description': description, **zeros}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(package / 'rviz/robot_6dof.rviz')], output='screen'),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('tool', default_value='parallel', choices=['parallel', 'none']),
        DeclareLaunchArgument('pose', default_value='home', choices=['home', 'zero']),
        OpaqueFunction(function=setup),
    ])
