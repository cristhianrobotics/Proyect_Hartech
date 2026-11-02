#!/usr/bin/env python3
"""Fundamentos de Robótica / HARTECH sin gripper: modelo propio vs KinPy.

Se reutilizan fkine_hartech, jacobian_6dof e ik_pose_6d existentes.
KinPy carga la cadena desde el URDF (base_link -> tool0).
Esta comparación es numérica: no mueve el robot ni usa Gazebo/RViz.
"""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof
from robot_control.ikine_pose_6d import ik_pose_6d

JOINTS = [f'joint_{i}' for i in range(1, 7)]
QMIN = np.deg2rad([-160, -95, -110, -150, -70, -160])
QMAX = np.deg2rad([160, 5, 110, 150, 70, 160])


def buscar_urdf():
    """Buscar el URDF del brazo que ya existe en robot_description."""
    carpetas = [Path.home() / 'Hartech_ws/src/robot_description/urdf']
    try:
        from ament_index_python.packages import get_package_share_directory
        carpetas.append(Path(get_package_share_directory('robot_description')) / 'urdf')
    except (ImportError, LookupError):
        pass
    for carpeta in carpetas:
        for nombre in ('robot_6dof.urdf', 'robot_6dof.urdf.xacro'):
            archivo = carpeta / nombre
            if archivo.is_file():
                return archivo
    raise FileNotFoundError(
        'No encontré el URDF de HARTECH en ~/Hartech_ws/src/robot_description/urdf/')


def cargar_xml(archivo):
    if archivo.suffix != '.xacro':
        return archivo.read_text(encoding='utf-8')
    try:
        import xacro
    except ImportError as exc:
        raise RuntimeError('Falta xacro. Instala: sudo apt install ros-humble-xacro') from exc
    return xacro.process_file(str(archivo)).toxml()


def error_pose(Td, Ta):
    pos_mm = 1000.0 * np.linalg.norm(Td[:3, 3] - Ta[:3, 3])
    Rerr = Td[:3, :3] @ Ta[:3, :3].T
    ang_deg = np.rad2deg(Rotation.from_matrix(Rerr).magnitude())
    return pos_mm, ang_deg


def main():
    try:
        import kinpy as kp
    except ImportError:
        print('Falta KinPy en el Python de ROS 2.')
        print('Instala con: python3 -m pip install --user kinpy')
        return

    archivo = buscar_urdf()
    cadena = kp.build_serial_chain_from_urdf(
        cargar_xml(archivo), 'tool0', root_link_name='base_link')
    orden = list(cadena.get_joint_parameter_names())
    if len(orden) != 6 or set(orden) != set(JOINTS):
        raise RuntimeError('Articulaciones diferentes entre URDF y HARTECH: ' + str(orden))
    indices = [JOINTS.index(j) for j in orden]

    def q_para_kinpy(q):
        return np.asarray(q, dtype=float)[indices]

    def q_desde_kinpy(q_kp):
        q_kp = np.asarray(q_kp, dtype=float)
        if q_kp.shape != (6,):
            raise ValueError('KinPy devolvió una configuración que no tiene seis articulaciones.')
        q = np.empty(6)
        for i, nombre in enumerate(orden):
            q[JOINTS.index(nombre)] = q_kp[i]
        return q

    def fk_kinpy(q):
        return np.asarray(cadena.forward_kinematics(q_para_kinpy(q)).matrix(), dtype=float)

    def jac_kinpy(q):
        # Las columnas deben corresponder a joint_1..joint_6, no al orden interno de KinPy.
        Jk = np.asarray(cadena.jacobian(q_para_kinpy(q)), dtype=float)
        if Jk.shape != (6, 6):
            raise ValueError(f'Jacobian KinPy inesperado: {Jk.shape}')
        J = np.empty((6, 6))
        for i, nombre in enumerate(orden):
            J[:, JOINTS.index(nombre)] = Jk[:, i]
        return J

    qhome = np.deg2rad([0, -50, 80, 0, -30, 0])
    qotra = np.deg2rad([30, -45, 60, 0, -20, 0])
    print('\n=== HARTECH SIN GRIPPER | MODELO PROPIO vs KinPy ===')
    print('Cadena: base_link -> tool0')
    print('URDF utilizado:', archivo)
    print('Articulaciones KinPy:', orden)

    for nombre, q in [('HOME', qhome), ('OTRA', qotra)]:
        Tpropia = fkine_hartech(q)
        Tkinpy = fk_kinpy(q)
        ep, er = error_pose(Tpropia, Tkinpy)
        Jpropio, Jkinpy = jacobian_6dof(q), jac_kinpy(q)
        print(f'\n--- {nombre} | q[°] = {np.rad2deg(q).round(3)} ---')
        print('FK propia XYZ [m]:', Tpropia[:3, 3].round(6))
        print('FK KinPy XYZ [m]:', Tkinpy[:3, 3].round(6))
        print(f'Diferencia FK: XYZ = {ep:.8f} mm | orientación = {er:.8f}°')
        print('Diferencia Jacobiano ||Jv_propio - Jv_KinPy|| =',
              f'{np.linalg.norm(Jpropio[:3, :] - Jkinpy[:3, :]):.3e}')
        print('Diferencia Jacobiano ||Jw_propio - Jw_KinPy|| =',
              f'{np.linalg.norm(Jpropio[3:, :] - Jkinpy[3:, :]):.3e}')

    # Misma pose deseada para ambos solucionadores (pose conocida alcanzable).
    Td = fkine_hartech(qotra)
    print('\n=== COMPARACIÓN DE CINEMÁTICA INVERSA (POSE COMPLETA) ===')
    print('Objetivo: pose de q=[30,-45,60,0,-20,0]°; semilla HOME.')

    qpropia, _, _ = ik_pose_6d(Td, qhome, QMIN, QMAX, qhome,
                                fkine_hartech, jacobian_6dof)
    if qpropia is None:
        print('IK propia: no encontró solución dentro de las tolerancias.')
    else:
        ep, er = error_pose(Td, fkine_hartech(qpropia))
        print('IK propia q [deg]:', np.rad2deg(qpropia).round(3))
        print(f'  Error: XYZ = {ep:.5f} mm | orientación = {er:.5f}°')
        print('  Respeta límites:', bool(np.all(qpropia >= QMIN) and np.all(qpropia <= QMAX)))

    # KinPy utiliza cuaterniones con orden [w,x,y,z].
    quat_xyzw = Rotation.from_matrix(Td[:3, :3]).as_quat()
    quat_wxyz = quat_xyzw[[3, 0, 1, 2]]
    objetivo = kp.Transform(rot=quat_wxyz, pos=Td[:3, 3])
    try:
        qkinpy = q_desde_kinpy(cadena.inverse_kinematics(objetivo, q_para_kinpy(qhome)))
        ep, er = error_pose(Td, fk_kinpy(qkinpy))
        print('IK KinPy q [deg]:', np.rad2deg(qkinpy).round(3))
        print(f'  Error: XYZ = {ep:.5f} mm | orientación = {er:.5f}°')
        print('  Respeta límites:', bool(np.all(qkinpy >= QMIN) and np.all(qkinpy <= QMAX)))
        if ep >= 1.0 or er >= 0.5:
            print('  AVISO: KinPy no alcanzó nuestras tolerancias para esta prueba.')
    except Exception as exc:
        print('KinPy no completó la IK:', type(exc).__name__, str(exc))

    print('\nPrueba numérica terminada. No se enviaron movimientos al robot.')


if __name__ == '__main__':
    main()
