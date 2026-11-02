from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description = Path(get_package_share_directory('robot_description'))
    tools = Path(get_package_share_directory('hartech_tools'))

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(description / 'launch/display_control.launch.py')
            ),
            launch_arguments={
                'model': str(tools / 'urdf/robot_with_tool.urdf.xacro')
            }.items()
        )
    ])