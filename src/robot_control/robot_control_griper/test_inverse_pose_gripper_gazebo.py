#!/usr/bin/env python3
"""HARTECH con gripper: cinemática inversa XYZ+RPY del TCP en Gazebo Classic.

Comparte la lógica 6D y las trayectorias con la versión sin gripper.
Los dedos se abren/cierran mediante gripper_controller por separado.
"""
from robot_control.test_inverse_pose_gazebo import ejecutar_gazebo
from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.jacobian_gripper import jacobian_gripper


def main():
    ejecutar_gazebo(fkine_gripper, jacobian_gripper, 'hartech_tcp')


if __name__ == '__main__':
    main()
