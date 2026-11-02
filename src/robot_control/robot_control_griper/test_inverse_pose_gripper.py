#!/usr/bin/env python3
"""HARTECH con gripper: IK completa XYZ + RPY del TCP en RViz."""
from robot_control.test_inverse_pose_6d import ejecutar_nodo
from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.jacobian_gripper import jacobian_gripper


def main():
    ejecutar_nodo(fkine_gripper, jacobian_gripper, 'hartech_tcp', True)


if __name__ == '__main__':
    main()
