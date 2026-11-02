#!/usr/bin/env python3
"""Crea el modelo Pinocchio de HARTECH + gripper a apertura FIJA (6 GDL).

Se obtiene del Xacro de hartech_tools instalado y se bloquean dedo izquierdo,
dedo derecho y rotor en una apertura elegida. Pinocchio incorpora al brazo
sus masas, centros de masa e inercias reales del URDF del gripper.
"""
import math
import tempfile
from pathlib import Path

import numpy as np
import pinocchio as pin
import xacro
from ament_index_python.packages import get_package_share_directory

JOINTS_BRAZO = [f'joint_{i}' for i in range(1, 7)]
JOINTS_GRIPPER = [
    'hartech_left_finger_joint',
    'hartech_right_finger_joint',
    'hartech_rotor_joint',
]


def cargar_modelo_con_gripper(apertura_mm=30.0):
    """Devuelve un modelo 6x6 con el gripper rígido a la apertura especificada.

    0..50 mm es la separación ENTRE almohadillas. No modela cierre/apertura
    durante la trayectoria ni fuerzas de contacto con piezas.
    """
    apertura_mm = float(apertura_mm)
    if not math.isfinite(apertura_mm) or not 0.0 <= apertura_mm <= 50.0:
        raise ValueError('La apertura debe estar entre 0 y 50 mm.')
    xacro_file = Path(get_package_share_directory('hartech_tools')) / 'urdf' / 'robot_with_tool.urdf.xacro'
    if not xacro_file.is_file():
        raise FileNotFoundError(f'No se encontró el Xacro del gripper: {xacro_file}')
    xml = xacro.process_file(str(xacro_file), mappings={'tool': 'parallel'}).toxml()
    # Generamos temporalmente el URDF completo. No cambiamos robot_description ni hartech_tools.
    with tempfile.TemporaryDirectory(prefix='hartech_gripper_din_') as tmp:
        urdf = Path(tmp) / 'hartech_con_gripper.urdf'
        urdf.write_text(xml, encoding='utf-8')
        modelo_completo = pin.buildModelFromUrdf(str(urdf))

    for joint in JOINTS_BRAZO + JOINTS_GRIPPER:
        if not modelo_completo.existJointName(joint):
            raise ValueError(f'Falta la articulación {joint} en el URDF del gripper.')

    # Al fijar los 3 joints del gripper, el brazo vuelve a nq=nv=6.
    q_fija = pin.neutral(modelo_completo)
    d = apertura_mm / 2000.0
    posiciones = {'hartech_left_finger_joint': d,
                  'hartech_right_finger_joint': d,
                  'hartech_rotor_joint': -100.0 * d}
    ids_fijos = []
    for joint, valor in posiciones.items():
        jid = modelo_completo.getJointId(joint)
        ids_fijos.append(jid)
        if modelo_completo.nqs[jid] != 1:
            raise ValueError(f'Joint inesperado para bloqueo: {joint}')
        q_fija[modelo_completo.idx_qs[jid]] = valor
    model = pin.buildReducedModel(modelo_completo, ids_fijos, q_fija)
    if model.nq != 6 or model.nv != 6:
        raise ValueError(f'Se esperaba 6 GDL tras bloquear la pinza; nq={model.nq}, nv={model.nv}.')
    orden = list(model.names)[1:]
    if orden != JOINTS_BRAZO:
        raise ValueError(f'Orden articular inesperado después del bloqueo: {orden}')
    return model
