#!/usr/bin/env python3
"""Análisis del Jacobiano 6x6 de HARTECH con gripper, en su TCP."""
from robot_control.test_singularities_6dof import analizar_singularidades
from robot_control_griper.jacobian_gripper import jacobian_gripper


def main():
    analizar_singularidades('CON GRIPPER (hartech_tcp)', jacobian_gripper,
                           'singularidades_hartech_con_gripper.png')


if __name__ == '__main__':
    main()
