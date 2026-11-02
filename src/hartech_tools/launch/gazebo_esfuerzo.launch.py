"""HARTECH con gripper: Gazebo Classic 11 para control articular por TORQUE.

Usa un launch y YAML independientes. NO modifica el control por posicion.
Las interfaces de los seis joints del brazo pasan a 'effort' solamente en
el URDF generado en memoria; los joints de la pinza conservan 'position'.
"""
import os
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

JOINTS = [f'joint_{i}' for i in range(1, 7)]


def urdf_para_esfuerzo(xml, paquetes, sim_joints):
    root = ET.fromstring(xml)
    sistemas = [s for s in root.findall('ros2_control') if s.get('name') == 'GazeboSystem']
    if len(sistemas) != 1:
        raise RuntimeError('No se encontro exactamente un ros2_control GazeboSystem para el brazo.')
    sistema = sistemas[0]
    for name in JOINTS:
        articulacion = next((j for j in sistema.findall('joint') if j.get('name') == name), None)
        if articulacion is None:
            raise RuntimeError(f'Falta {name} en ros2_control del brazo.')
        comandos = articulacion.findall('command_interface')
        if len(comandos) != 1 or comandos[0].get('name') != 'position':
            raise RuntimeError(f'Interfaz inesperada en {name}; no se modifico el Xacro original.')
        comando = comandos[0]
        comando.set('name', 'effort')
        for hijo in list(comando):
            comando.remove(hijo)  # Eran limites de POSICION, no de torque.
        esfuerzo = float(sim_joints[name]['effort'])
        ET.SubElement(comando, 'param', {'name': 'min'}).text = str(-esfuerzo)
        ET.SubElement(comando, 'param', {'name': 'max'}).text = str(esfuerzo)

    # Reemplazar los paquetes de mallas por rutas absolutas, como en el launch existente.
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            prefix, _, relative = uri[10:].partition('/')
            if prefix in paquetes:
                path = paquetes[prefix] / relative
                if not path.is_file():
                    raise FileNotFoundError(f'No se encuentra el mesh: {path}')
                mesh.set('filename', str(path))
    return ET.tostring(root, encoding='unicode')



def crear_mundo_arranque_sin_gravedad(ruta_mundo):
    """Crear una copia TEMPORAL del mundo, con gravedad nula solo al arrancar.

    Gazebo sigue ejecutandose para que /controller_manager pueda cargar y
    activar los controladores. El PD restaura 0,0,-9.81 cuando esta listo.
    No modifica el .world original de HARTECH.
    """
    arbol = ET.parse(str(ruta_mundo))
    mundo = arbol.getroot().find('world')
    if mundo is None:
        raise RuntimeError('El archivo de mundo no contiene un elemento <world>.')
    nombre_mundo = mundo.get('name')
    if not nombre_mundo:
        raise RuntimeError('El mundo SDF no tiene nombre.')
    gravedad = mundo.find('gravity')
    if gravedad is None:
        gravedad = ET.SubElement(mundo, 'gravity')
    gravedad.text = '0 0 0'
    with tempfile.NamedTemporaryFile(
        mode='wb', prefix='hartech_pd_esfuerzo_', suffix='.world', delete=False
    ) as temporal:
        arbol.write(temporal, encoding='utf-8', xml_declaration=True)
        return temporal.name, nombre_mundo

def gazebo_resource_path(existing):
    paths = [p for p in existing.split(os.pathsep) if p]
    for loc in ('/usr/share/gazebo-11', '/usr/share/gazebo'):
        if (Path(loc) / 'media/rtshaderlib').is_dir() and loc not in paths:
            paths.append(loc)
    return os.pathsep.join(paths)


def setup(context):
    desc = Path(get_package_share_directory('robot_description'))
    addon = Path(get_package_share_directory('hartech_tools'))
    gazebo_pkg = Path(get_package_share_directory('gazebo_ros'))
    pose = LaunchConfiguration('pose').perform(context)
    if pose != 'home':
        raise ValueError('Esta primera prueba de esfuerzo solo admite pose:=home.')
    yaml_esfuerzo = addon / 'config/controllers_esfuerzo.yaml'
    sim_file = desc / 'config/gazebo_simulation.yaml'
    config_sim = yaml.safe_load(sim_file.read_text(encoding='utf-8'))
    mundo_temporal, nombre_mundo = crear_mundo_arranque_sin_gravedad(
        Path(LaunchConfiguration('world').perform(context)))
    modelo = addon / 'urdf/robot_with_tool.gazebo.xacro'
    doc = xacro.process_file(str(modelo), mappings={
        'parameters_file': str(desc / 'config/robot_parameters.yaml'),
        'simulation_file': str(sim_file),
        'controllers_file': str(yaml_esfuerzo),
        'pose': pose, 'tool': 'parallel',
    })
    robot_description = urdf_para_esfuerzo(
        doc.toxml(), {'robot_description': desc, 'hartech_tools': addon}, config_sim['joints'])

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', output='screen',
               parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                            'use_sim_time': True}])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_pkg / 'launch/gazebo.launch.py')),
        launch_arguments={
            'world': mundo_temporal,
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true', 'pause': 'false',
        }.items())
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', '/robot_description', '-entity', 'robot_6dof', '-timeout', '120.0'])

    def spawner(nombre):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[nombre, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '120', '--switch-timeout', '30'])

    js = spawner('joint_state_broadcaster')
    pinza = spawner('gripper_controller')
    esfuerzo = spawner('arm_effort_controller')
    pd = Node(package='robot_control', executable='test_pd_gravedad_gripper_gazebo',
              name='pd_gravedad_gripper_gazebo', output='screen', emulate_tty=True,
              arguments=['--modo', LaunchConfiguration('modo'),
                         '--gazebo-world', nombre_mundo],
              parameters=[{'use_sim_time': True}])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', str(desc / 'rviz/robot_6dof.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    def continuar(acciones, mensaje):
        def handler(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg=mensaje), EmitEvent(event=Shutdown(reason=mensaje))]
            return acciones
        return handler

    siguiente = [LogInfo(msg='Control esfuerzo activo. PD restaurara gravedad normal cuando este listo.'), rviz]
    if LaunchConfiguration('autostart_pd').perform(context) == 'true':
        siguiente.insert(1, pd)
    return [
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path(
            os.environ.get('GAZEBO_RESOURCE_PATH', ''))),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=continuar([js], 'Fallo al insertar HARTECH en Gazebo.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=js,
            on_exit=continuar([pinza], 'Fallo el publicador de estados.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=pinza,
            on_exit=continuar([esfuerzo], 'Fallo el controlador del gripper.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=esfuerzo,
            on_exit=continuar(siguiente, 'Fallo arm_effort_controller.'))),
        LogInfo(msg='Arranque temporal SIN GRAVEDAD para prevenir caida antes de activar PD.'),
        rsp, gazebo, spawn,
    ]


def generate_launch_description():
    paquete = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('pose', default_value='home', choices=['home']),
        DeclareLaunchArgument('modo', default_value='critico', choices=['sub', 'critico', 'sobre']),
        DeclareLaunchArgument('autostart_pd', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value=str(paquete / 'worlds/robot_lab.world')),
        OpaqueFunction(function=setup),
    ])
