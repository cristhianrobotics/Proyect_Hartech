#!/usr/bin/env python3
"""Cinemática directa de HARTECH hasta hartech_tcp (gripper paralelo)."""
import numpy as np
from fkine_functions import fkine_hartech


def fkine_gripper(q):
    """Matriz homogénea base_link -> hartech_tcp, con q de 6 articulaciones [rad]."""
    T_tool0 = fkine_hartech(q)  # DH original verificada con URDF
    T_tool0_tcp = np.eye(4)
    T_tool0_tcp[2, 3] = 0.100  # TCP a 100 mm en el +Z local de tool0
    return T_tool0 @ T_tool0_tcp
