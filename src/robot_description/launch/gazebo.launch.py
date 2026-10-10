"""Gazebo Classic 11 with one controller manager supplied by the model plugin."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def prepare_description(document, package):
    """Remove comments rejected by Humble's plugin argument parser; resolve meshes."""
    root = ET.fromstring(document.toxml())
    prefix = 'package://robot_description/'
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith(prefix):
            path = package / filename[len(prefix):]
            if not path.is_file():
                raise FileNotFoundError(f'Missing robot mesh: {path}')
            mesh.set('filename', str(path))
    return ET.tostring(root, encoding='unicode')


def gazebo_resource_path(existing):
    """Keep existing resources and add the installed Classic 11 shader resources."""
    paths = [item for item in existing.split(os.pathsep) if item]
    for location in ('/usr/share/gazebo-11', '/usr/share/gazebo'):
        if (Path(location) / 'media/rtshaderlib').is_dir() and location not in paths:
            paths.append(location)
    return os.pathsep.join(paths)


def setup(context):
    package = Path(get_package_share_directory('robot_description'))
    gazebo_package = Path(get_package_share_directory('gazebo_ros'))
    pose = LaunchConfiguration('pose').perform(context)
    if pose not in ('home', 'zero'):
        raise ValueError('pose must be home or zero')
    controllers = package / 'config/gazebo_controllers.yaml'
    model = package / 'urdf/robot_6dof.gazebo.xacro'
    document = xacro.process_file(str(model), mappings={
        'parameters_file': str(package / 'config/robot_parameters.yaml'),
        'simulation_file': str(package / 'config/gazebo_simulation.yaml'),
        'controllers_file': str(controllers),
        'pose': pose,
    })
    description = prepare_description(document, package)

    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'robot_description': ParameterValue(description, value_type=str),
            'use_sim_time': True,
        }],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_package / 'launch/gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
            'pause': 'false',
        }.items(),
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', '/robot_description', '-entity', 'robot_6dof',
                   '-timeout', '120.0'],
    )
    state_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120', '--switch-timeout', '30'],
    )
    arm_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['arm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120', '--switch-timeout', '30'],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(package / 'rviz/robot_6dof.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    def continue_if_success(next_actions, message):
        def handler(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg=message), EmitEvent(event=Shutdown(reason=message))]
            return next_actions
        return handler

    return [
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path(
            os.environ.get('GAZEBO_RESOURCE_PATH', ''),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=continue_if_success([state_spawner], 'Robot spawn failed.'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=state_spawner,
            on_exit=continue_if_success([arm_spawner], 'Joint state broadcaster failed.'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner,
            on_exit=continue_if_success([
                LogInfo(msg='Gazebo controllers are active. Ready for trajectories.'), rviz,
            ], 'Arm trajectory controller failed.'),
        )),
        publisher, gazebo, spawn,
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('pose', default_value='home', choices=['home', 'zero']),
        DeclareLaunchArgument('world', default_value=str(package / 'worlds/robot_lab.world')),
        OpaqueFunction(function=setup),
    ])
