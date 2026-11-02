import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    py_modules=['markers','fkine_functions','ikine_functions'],
    zip_safe=True,
    maintainer='cristhian',
    maintainer_email='jhonquispe7164@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_forward_kinematic = robot_control.test_forward_kinematic:main',
            'test_inverse_kinematics = robot_control.test_inverse_kinematics:main',
            'test_forward_gazebo = robot_control.test_forward_gazebo:main',
            'test_inverse_gazebo = robot_control.test_inverse_gazebo:main',
            'test_dynamics = robot_control.test_dynamics:main',
            'test_forward_dynamics = robot_control.test_forward_dynamics:main',
            # HARTECH con gripper: nodos nuevos, sin alterar los anteriores
            'test_forward_kinematic_gripper = robot_control_griper.test_forward_kinematic_gripper:main',
            'test_inverse_kinematics_gripper = robot_control_griper.test_inverse_kinematics_gripper:main',
            'test_forward_gripper_gazebo = robot_control_griper.test_forward_gripper_gazebo:main',
            'test_inverse_gripper_gazebo = robot_control_griper.test_inverse_gripper_gazebo:main',
            'test_dynamics_gripper = robot_control_griper.test_dynamics_gripper:main',
            'test_forward_dynamics_gripper = robot_control_griper.test_forward_dynamics_gripper:main',
            # Jacobianos
            'test_jacobian_6dof = robot_control.test_jacobian_6dof:main',
            'test_jacobian_gripper = robot_control_griper.test_jacobian_gripper:main',
            # Cinemática inversa XYZ + RPY en rviz
            'test_inverse_pose_6d = robot_control.test_inverse_pose_6d:main',
            'test_inverse_pose_gripper = robot_control_griper.test_inverse_pose_gripper:main',
            # Cinemática inversa XYZ + RPY en Gazebo
            'test_inverse_pose_gazebo = robot_control.test_inverse_pose_gazebo:main',
            'test_inverse_pose_gripper_gazebo = robot_control_griper.test_inverse_pose_gripper_gazebo:main',
            # Análisis de singularidades
            'test_singularities_6dof = robot_control.test_singularities_6dof:main',
            'test_singularities_gripper = robot_control_griper.test_singularities_gripper:main',
            # Laboratorio 6 - Control cinemático diferencial
            'test_diffkine_hartech = robot_control.test_diffkine_hartech:main',
            # KinPy: cinemática directa e inversa simbólica con SymPy
            'comparar_kinpy_hartech = robot_control.comparar_kinpy_hartech:main',
            # Lab 6 - Control cinemático diferencial en Gazebo
            'test_diffkine_gazebo_hartech = robot_control.test_diffkine_gazebo_hartech:main',
            # Control diferencial de posición y orientación
            'test_diffkine_pose_gazebo_hartech = robot_control.test_diffkine_pose_gazebo_hartech:main',
            # Seguimiento de trayectoria cartesiana
            'test_trayectoria_cartesiana_gazebo_hartech = robot_control.test_trayectoria_cartesiana_gazebo_hartech:main',
            # LAB 6 - CONTROL CINEMÁTICO DIFERENCIAL CON GRIPPER
            'test_diffkine_gripper = robot_control_griper.test_diffkine_gripper:main',
            'test_diffkine_gripper_gazebo = robot_control_griper.test_diffkine_gripper_gazebo:main',
            'test_diffkine_pose_gripper_gazebo = robot_control_griper.test_diffkine_pose_gripper_gazebo:main',
            'test_trayectoria_cartesiana_gripper_gazebo = robot_control_griper.test_trayectoria_cartesiana_gripper_gazebo:main',
            'comparar_kinpy_gripper = robot_control_griper.comparar_kinpy_gripper:main',
            'test_trayectoria_cartesiana_gripper_gazebo_v2 = robot_control_griper.test_trayectoria_cartesiana_gripper_gazebo_v2:main',

            # Prueba de control dinamico con gripper
            'test_pd_gravedad_gripper = robot_control_griper.test_pd_gravedad_gripper:main',

            #Control de torque con gripper
            'test_pd_gravedad_gripper_gazebo = robot_control_griper.test_pd_gravedad_gripper_gazebo:main',

            # Lab 7 - Control dinámico HARTECH con gripper
            'test_dinamica_inversa_gripper_aba = robot_control_griper.test_dinamica_inversa_gripper_aba:main',
            'test_control_dinamico_gripper_gazebo = robot_control_griper.test_control_dinamico_gripper_gazebo:main',
        ],
    },
)
