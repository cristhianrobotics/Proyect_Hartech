"""Jacobiano geométrico 6x6 para el TCP del gripper HARTECH.

Respeta Jw_TCP=Jw_tool0; Jv_TCP=Jv_tool0 + Jw x r,
donde r es el vector tool0->TCP expresado en base_link.
"""
import numpy as np
from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof
from robot_control_griper.fkine_gripper import fkine_gripper


def jacobian_gripper(q):
    q = np.asarray(q, dtype=float)
    J = jacobian_6dof(q)
    p_tool0 = fkine_hartech(q)[:3, 3]
    p_tcp = fkine_gripper(q)[:3, 3]
    r = p_tcp - p_tool0
    J_tcp = J.copy()
    J_tcp[:3, :] += np.cross(J[3:, :].T, r).T
    return J_tcp
