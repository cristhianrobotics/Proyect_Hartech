#!/usr/bin/env python3
"""Lab 6 UR5 adaptado a HARTECH CON GRIPPER: control cinemático de XYZ en RViz.

Reutiliza el FK y el Jacobiano 6x6 YA existentes. No modifica esos módulos.
No envía comandos a Gazebo: solo publica /joint_states para visualización.
"""
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.jacobian_gripper import jacobian_gripper


def crear_marcador(node, marcador_id, rgb):
    """Marcador esférico (equivale a los dos BallMarker del Lab 6).

    Se construye aquí para no modificar el markers.py que ya usa HARTECH.
    """
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = 'control_diferencial_lab6_gripper'
    m.id = marcador_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.045
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    return m


def main():
    rclpy.init()
    node = rclpy.create_node('control_cinematico_gripper_hartech')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    pub_markers = node.create_publisher(Marker, '/visualization_marker', 10)
    # No se requiere rclpy.spin() porque este nodo solo publica mensajes.

    # Mismos archivos de registro del Lab 6, dentro de nuestro workspace.
    ruta = Path.home() / 'Hartech_ws' / 'resultados_lab6'
    ruta.mkdir(parents=True, exist_ok=True)

    # Posición objetivo del HARTECH y configuración inicial HOME.
    xd = fkine_gripper(np.deg2rad([30, -45, 60, 0, -20, 0]))[:3, 3].copy()
    q = np.deg2rad([0, -50, 80, 0, -30, 0]).astype(float)
    qmin = np.deg2rad([-160, -95, -110, -150, -70, -160])
    qmax = np.deg2rad([160, 5, 110, 150, 70, 160])

    jstate = JointState()
    jstate.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
                   'hartech_left_finger_joint']
    marcador_actual = crear_marcador(node, 0, (1.0, 0.0, 0.0))
    marcador_deseado = crear_marcador(node, 1, (0.0, 1.0, 0.0))
    marcador_deseado.pose.position.x, marcador_deseado.pose.position.y, marcador_deseado.pose.position.z = map(float, xd)

    # Parámetros originales del laboratorio: k=0.5, integración Euler.
    freq = 50
    dt = 1.0 / freq
    k = 0.5
    print('\n=== Lab 6 HARTECH | Control cinemático diferencial de XYZ del TCP ===')
    print('Solo RViz, con gripper; cinemática directa y Jacobiano existentes.')
    print('XYZ deseada [m]:', xd, '| k:', k, '| dt:', dt)

    try:
        with (ruta / 'xcurrent_gripper.txt').open('w') as fxc, \
             (ruta / 'xdesired_gripper.txt').open('w') as fxd, \
             (ruta / 'q_gripper.txt').open('w') as fq:
            for paso in range(2000):
                if not rclpy.ok():
                    break
                # Mismo núcleo de test_diffkine.py del UR5:
                T = fkine_gripper(q)
                x = T[0:3, 3]
                J = jacobian_gripper(q)[:3, :]  # Jacobiano DE POSICIÓN (3x6)
                e = xd - x
                qdot = np.linalg.pinv(J).dot(k * e)
                q_nueva = q + qdot * dt          # Euler

                # Protección de límites articulares del HARTECH.
                if np.any(q_nueva < qmin) or np.any(q_nueva > qmax):
                    print('Detenido: la siguiente referencia excede los límites articulares.')
                    break
                q = q_nueva
                x = fkine_gripper(q)[:3, 3]
                error_mm = np.linalg.norm(xd - x) * 1000.0

                fxc.write(' '.join(map(str, x)) + '\n')
                fxd.write(' '.join(map(str, xd)) + '\n')
                fq.write(' '.join(map(str, q)) + '\n')

                sello = node.get_clock().now().to_msg()
                jstate.header.stamp = sello
                jstate.position = q.tolist() + [0.015]  # Apertura fija 30 mm en RViz
                pub.publish(jstate)
                marcador_actual.header.stamp = sello
                marcador_deseado.header.stamp = sello
                marcador_actual.pose.position.x, marcador_actual.pose.position.y, marcador_actual.pose.position.z = map(float, x)
                pub_markers.publish(marcador_actual)
                pub_markers.publish(marcador_deseado)

                if paso % 25 == 0 or error_mm < 1.0:
                    print(f'Iteración {paso+1:4d} | Error XYZ: {error_mm:.4f} mm')
                if error_mm < 1.0:
                    print('Objetivo alcanzado. q final [deg]:', np.round(np.rad2deg(q), 3))
                    break
                time.sleep(dt)
            else:
                print('Se alcanzó el máximo de iteraciones; revisa el error restante.')
    except KeyboardInterrupt:
        print('Detenido por el usuario.')
    finally:
        print('Registros guardados en:', ruta)
        time.sleep(0.1)  # Permite entregar los ultimos mensajes publicados.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
