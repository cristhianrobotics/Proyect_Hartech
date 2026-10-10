"""Display the HARTECH robot in RViz without joint sliders."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    package = Path(get_package_share_directory('robot_description'))
    model = Path(LaunchConfiguration('model').perform(context))
    description = (xacro.process_file(str(model)).toxml()
                   if model.suffix == '.xacro' else model.read_text(encoding='utf-8'))
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(package / 'rviz/robot_6dof.rviz')], output='screen'),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(package / 'urdf/robot_6dof.urdf.xacro')),
        OpaqueFunction(function=setup),
    ])