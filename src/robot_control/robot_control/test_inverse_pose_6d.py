#!/usr/bin/env python3
"""HARTECH: cinemática inversa de pose completa XYZ + RPY en RViz, sin sliders.

Este nodo reutiliza la misma ejecución para tool0 o para hartech_tcp.
Las IK de posición que ya funcionan se mantienen intactas.
"""
from pathlib import Path
import threading
import time

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from markers import FrameMarker
from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof, singular_values_scaled
from robot_control.ikine_pose_6d import (JOINTS, rpy_a_rotacion,
    rotacion_a_rpy, rotacion_a_cuaternion_xyzw, ik_pose_6d)


def ejecutar_nodo(fk, jac, referencia='tool0', con_gripper=False):
    """Publica q1..q6; en la versión con gripper añade apertura fija 30 mm."""
    ruta = Path(get_package_share_directory('robot_description'))
    config = yaml.safe_load((ruta / 'config/robot_parameters.yaml').read_text())
    qmin = np.array([float(config['joints'][j]['lower']) for j in JOINTS])
    qmax = np.array([float(config['joints'][j]['upper']) for j in JOINTS])
    qhome = np.array([float(config['joints'][j]['home']) for j in JOINTS])
    if np.any(qmin >= qmax):
        raise ValueError('Límites articulares incorrectos en robot_parameters.yaml.')

    rclpy.init()
    nombre = 'InversePoseGripperRViz' if con_gripper else 'InversePoseHartechRViz'
    node = rclpy.create_node(nombre)
    pub = node.create_publisher(JointState, '/joint_states', 10)
    marker = FrameMarker(node)
    lock = threading.Lock()
    q = qhome.copy()

    def pose_marcador(q_pose):
        T = fk(q_pose)
        quat = rotacion_a_cuaternion_xyzw(T[:3, :3])
        # FrameMarker.setPose espera [x, y, z, qw, qx, qy, qz].
        return np.r_[T[:3, 3], quat[3], quat[:3]]

    def mostrar_pose(nombre_pose, T):
        print(f'\n{nombre_pose}: XYZ [m] =', np.round(T[:3, 3], 6))
        print('               RPY [deg] =', np.round(rotacion_a_rpy(T[:3, :3]), 4))

    print(f'\n=== IK POSE 6D HARTECH - RViz - {referencia} ===')
    print('Entrada: x y z [m] roll pitch yaw [grados]')
    print('Convención: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)')
    print("Comandos especiales: 'home', 'otra', 'actual', 'salir'.")
    print('La apertura del gripper permanece fija en 30 mm.' if con_gripper
          else 'Se controla el frame tool0, sin gripper.')
    mostrar_pose('HOME de referencia', fk(qhome))
    qotra = np.deg2rad([30, -45, 60, 0, -20, 0])
    mostrar_pose('OTRA pose alcanzable', fk(qotra))
    print('\nUsa primero `otra` y después `home` para comprobar una pose 6D alcanzable.\n')
    marker.setPose(pose_marcador(qhome))

    def leer_pose():
        nonlocal q
        while rclpy.ok():
            try:
                entrada = input('XYZ[m] RPY[deg] > ').strip().lower()
                if entrada in ('salir', 'quit', 'exit'):
                    if rclpy.ok():
                        rclpy.shutdown()
                    break
                if entrada == 'actual':
                    with lock:
                        actual = q.copy()
                    mostrar_pose('POSE ACTUAL', fk(actual))
                    continue
                if entrada in ('home', 'otra'):
                    Tdes = fk(qhome if entrada == 'home' else qotra)
                else:
                    valores = [float(v) for v in entrada.replace(',', ' ').split()]
                    if len(valores) != 6 or not np.all(np.isfinite(valores)):
                        print('Debes ingresar 6 números finitos: x y z roll pitch yaw.')
                        continue
                    Tdes = np.eye(4)
                    Tdes[:3, 3] = valores[:3]
                    Tdes[:3, :3] = rpy_a_rotacion(valores[3:])
                with lock:
                    qinicial = q.copy()
                print('\nBuscando solución de posición + orientación con límites articulares...')
                qsol, historial, errores = ik_pose_6d(
                    Tdes, qinicial, qmin, qmax, qhome, fk, jac)
                if qsol is None:
                    print('SIN SOLUCIÓN verificada para esta pose y las semillas probadas.')
                    print('Mejor error aproximado: XYZ = %.3f mm, orientación = %.3f°' %
                          (errores[0]*1000, np.rad2deg(errores[1])))
                    print('HARTECH mantiene la configuración anterior.\n')
                    continue
                Tfinal = fk(qsol)
                epos = np.linalg.norm(Tdes[:3, 3] - Tfinal[:3, 3])
                eang = np.rad2deg(errores[1])
                if (np.any(qsol < qmin - 1e-9) or np.any(qsol > qmax + 1e-9)
                    or epos >= .001 or eang >= 0.5):
                    print('ADVERTENCIA: solución fuera de límites o tolerancias; no publicada.')
                    continue
                with lock:
                    q = qsol.copy()
                    marcador = pose_marcador(q)
                marker.setPose(marcador)
                mostrar_pose('POSE DESEADA', Tdes)
                mostrar_pose('POSE ALCANZADA', Tfinal)
                print('q [deg]:', np.round(np.rad2deg(qsol), 3))
                print('Error XYZ: %.4f mm | Error orientación: %.4f°' % (epos*1000, eang))
                s = singular_values_scaled(jac(qsol), length=0.3)
                print('Razón singular σmin/σmax (L=0.3m): %.5f' % (s[-1]/s[0]))
                if s[-1]/s[0] < 0.02:
                    print('AVISO: esta configuración puede estar cerca de una singularidad.')
                print('Solución validada y publicada en RViz.\n')
            except ValueError as exc:
                print('Entrada inválida:', exc)
            except (EOFError, KeyboardInterrupt):
                if rclpy.ok():
                    rclpy.shutdown()
                break
            except Exception as exc:
                print('Error:', exc)

    hilo_ros = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    hilo_ros.start()
    hilo_entrada = threading.Thread(target=leer_pose, daemon=True)
    hilo_entrada.start()

    jstate = JointState()
    jstate.name = JOINTS + (['hartech_left_finger_joint'] if con_gripper else [])
    try:
        while rclpy.ok():
            with lock:
                qactual = q.copy()
            jstate.header.stamp = node.get_clock().now().to_msg()
            jstate.position = qactual.tolist() + ([0.015] if con_gripper else [])
            pub.publish(jstate)
            marker.publish()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        hilo_ros.join(timeout=2.0)
        node.destroy_node()


def main():
    ejecutar_nodo(fkine_hartech, jacobian_6dof, 'tool0', False)


if __name__ == '__main__':
    main()
