#!/usr/bin/env python3
"""HARTECH sin gripper: cinemática inversa XYZ+RPY y movimiento en Gazebo Classic.

El mismo ejecutor se reutiliza para hartech_tcp con el nodo del gripper.
El modelo matemático trabaja en base_link y q1..q6, sin controlar la pinza.
"""
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof, singular_values_scaled
from robot_control.ikine_pose_6d import (
    JOINTS, EPS_POS, EPS_ANG, ik_pose_6d, rpy_a_rotacion,
    rotacion_a_rpy, errores_pose,
)

TIMEOUT = 30.0
FRACCION_VELOCIDAD = 0.35
TOL_ARTICULAR = np.deg2rad(1.0)


def duracion_ros(segundos):
    ns = round(float(segundos) * 1e9)
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def esperar(futuro, segundos):
    """Espera una respuesta de ROS 2 mientras el nodo gira en otro hilo."""
    limite = time.monotonic() + float(segundos)
    while rclpy.ok() and not futuro.done():
        if time.monotonic() >= limite:
            raise TimeoutError('Se agotó el tiempo de espera de Gazebo.')
        time.sleep(0.05)
    if not futuro.done() or futuro.result() is None:
        raise RuntimeError('ROS 2 se interrumpió o no devolvió respuesta.')
    return futuro.result()


def guardar_error(historial, referencia):
    """Guardar convergencia de la IK 6D sin abrir ventana gráfica."""
    if not historial:
        return
    nombre = ('error_ik_pose_gripper_gazebo_' if referencia == 'hartech_tcp'
              else 'error_ik_pose_gazebo_')
    nombre += datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.png'
    plt.figure()
    plt.plot(np.asarray(historial)*1000, '.-')
    plt.xlabel('Iteración')
    plt.ylabel('Error combinado escalado [mm]')
    plt.title(f'Convergencia IK 6D - {referencia}')
    plt.grid(True)
    plt.savefig(nombre, dpi=180, bbox_inches='tight')
    plt.close()
    print('Gráfica guardada:', nombre)


def ejecutar_gazebo(fk, jac, referencia='tool0'):
    """Resuelve pose 6D y mueve solamente las seis articulaciones del brazo."""
    ruta = Path(get_package_share_directory('robot_description'))
    config = yaml.safe_load((ruta / 'config/robot_parameters.yaml').read_text())
    sim = yaml.safe_load((ruta / 'config/gazebo_simulation.yaml').read_text())
    qmin = np.array([float(config['joints'][j]['lower']) for j in JOINTS])
    qmax = np.array([float(config['joints'][j]['upper']) for j in JOINTS])
    qhome = np.array([float(config['joints'][j]['home']) for j in JOINTS])
    velocidades = np.array([float(sim['joints'][j]['velocity']) for j in JOINTS])
    if (not np.all(np.isfinite(qmin)) or not np.all(np.isfinite(qmax))
            or np.any(qmin >= qmax) or not np.all(np.isfinite(velocidades))
            or np.any(velocidades <= 0)):
        raise ValueError('Parámetros articulares o de velocidad inválidos.')

    rclpy.init()
    nombre = ('InversePoseGripperGazebo' if referencia == 'hartech_tcp'
              else 'InversePoseHartechGazebo')
    node = rclpy.create_node(nombre, parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    cliente = ActionClient(node, FollowJointTrajectory,
                           '/arm_controller/follow_joint_trajectory')
    estados = {}
    bloqueo = threading.Lock()
    ultimo_estado = [0.0]
    objetivo_activo = [None]

    def recibir_estado(msg):
        with bloqueo:
            for nombre_art, valor in zip(msg.name, msg.position):
                estados[nombre_art] = valor
            if all(j in estados for j in JOINTS):
                ultimo_estado[0] = time.monotonic()

    sub = node.create_subscription(JointState, '/joint_states', recibir_estado, 10)
    hilo = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    hilo.start()

    def estado_actual():
        with bloqueo:
            if not all(j in estados for j in JOINTS):
                return None
            q = np.array([estados[j] for j in JOINTS], dtype=float)
            antiguedad = time.monotonic() - ultimo_estado[0]
        if not np.all(np.isfinite(q)) or antiguedad > 3.0:
            return None
        return q

    def mostrar_pose(etiqueta, T):
        print(f'{etiqueta} XYZ [m]:', np.round(T[:3, 3], 6))
        print(f'{etiqueta} RPY [deg]:', np.round(rotacion_a_rpy(T[:3, :3]), 4))

    otra = np.deg2rad([30, -45, 60, 0, -20, 0])
    try:
        print('Esperando /arm_controller/follow_joint_trajectory ...')
        if not cliente.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('No se encontró arm_controller. Inicia Gazebo.')
        limite = time.monotonic() + TIMEOUT
        while rclpy.ok() and estado_actual() is None and time.monotonic() < limite:
            time.sleep(0.1)
        if estado_actual() is None:
            raise TimeoutError('No se recibieron estados articulares de Gazebo.')

        print(f'\n=== HARTECH - IK POSE 6D - GAZEBO - {referencia} ===')
        print('Entrada: X Y Z [m] Roll Pitch Yaw [grados]')
        print('Convención: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)')
        print("Comandos: 'home', 'otra', 'actual', 'salir'.")
        if referencia == 'hartech_tcp':
            print('Se controla el TCP. La apertura del gripper se controla aparte.')
        mostrar_pose('HOME alcanzable:', fk(qhome))
        mostrar_pose('OTRA alcanzable:', fk(otra))
        print("\nPrueba 'otra' y luego 'home' para comenzar con poses conocidas.\n")

        while rclpy.ok():
            try:
                entrada = input('XYZ[m] RPY[deg] > ').strip().lower()
                if entrada in ('salir', 'exit', 'quit'):
                    break
                qactual = estado_actual()
                if qactual is None:
                    print('ERROR: no se reciben posiciones articulares recientes.')
                    continue
                if entrada == 'actual':
                    mostrar_pose('POSE ACTUAL:', fk(qactual))
                    continue
                if entrada in ('home', 'otra'):
                    Tdes = fk(qhome if entrada == 'home' else otra)
                else:
                    valores = [float(x) for x in entrada.replace(',', ' ').split()]
                    if len(valores) != 6 or not np.all(np.isfinite(valores)):
                        print('Ingresa seis números finitos: x y z roll pitch yaw.')
                        continue
                    Tdes = np.eye(4)
                    Tdes[:3, 3] = valores[:3]
                    Tdes[:3, :3] = rpy_a_rotacion(valores[3:])

                print('\nSemilla real [deg]:', np.round(np.rad2deg(qactual), 2))
                print('Resolviendo XYZ + orientación con límites articulares...')
                qsol, historial, errores = ik_pose_6d(
                    Tdes, qactual, qmin, qmax, qhome, fk, jac)
                if qsol is None:
                    print('No se encontró una solución verificada para esta pose.')
                    print('Mejor error: XYZ %.3f mm | orientación %.3f°\n' %
                          (errores[0]*1000, np.rad2deg(errores[1])))
                    continue
                Tcalculada = fk(qsol)
                ep, er = errores_pose(Tdes, Tcalculada)
                epos = float(np.linalg.norm(ep))
                eang = float(np.linalg.norm(er))
                if (np.any(qsol < qmin - 1e-9) or np.any(qsol > qmax + 1e-9)
                        or epos >= EPS_POS or eang >= EPS_ANG):
                    print('Solución rechazada: excede límites o tolerancias.')
                    continue
                print('\nq solución [deg]:', np.round(np.rad2deg(qsol), 3))
                mostrar_pose('POSE DESEADA:', Tdes)
                mostrar_pose('POSE CALCULADA:', Tcalculada)
                print('Error IK: XYZ %.4f mm | orientación %.4f°' %
                      (epos*1000, np.rad2deg(eang)))
                guardar_error(historial, referencia)
                sv = singular_values_scaled(jac(qsol), length=0.3)
                razon = sv[-1]/sv[0]
                print('Razón singular σmin/σmax (L=0.3m): %.5f' % razon)
                if razon < 0.02:
                    print('AVISO: configuración posiblemente cercana a singularidad.')

                # Puede haber otra solución distinta de la postura actual, aun
                # cuando el TCP ya esté en la pose pedida. Si ya se alcanzó la
                # pose deseada, no enviamos un movimiento innecesario.
                ep0, er0 = errores_pose(Tdes, fk(qactual))
                if (np.linalg.norm(ep0) < EPS_POS and
                        np.linalg.norm(er0) < EPS_ANG):
                    print('El TCP ya está dentro de las tolerancias deseadas.\n')
                    continue

                delta = np.abs(qsol - qactual)
                duracion = max(0.6, 0.1 + 1.2*1.5*np.max(
                    delta/(FRACCION_VELOCIDAD*velocidades)))
                objetivo = FollowJointTrajectory.Goal()
                objetivo.trajectory.joint_names = JOINTS
                punto_inicio = JointTrajectoryPoint()
                punto_inicio.positions = qactual.tolist()
                punto_inicio.velocities = [0.0]*6
                punto_inicio.time_from_start = duracion_ros(0.1)
                punto_final = JointTrajectoryPoint()
                punto_final.positions = qsol.tolist()
                punto_final.velocities = [0.0]*6
                punto_final.time_from_start = duracion_ros(duracion)
                objetivo.trajectory.points = [punto_inicio, punto_final]
                objetivo.goal_time_tolerance = duracion_ros(3.0)

                print(f'Enviando trayectoria (duración calculada {duracion:.2f} s)...')
                handle = esperar(cliente.send_goal_async(objetivo), TIMEOUT)
                if not handle.accepted:
                    print('El controlador rechazó la trayectoria.\n')
                    continue
                objetivo_activo[0] = handle
                try:
                    respuesta = esperar(handle.get_result_async(), duracion + TIMEOUT)
                except TimeoutError:
                    print('Sin respuesta de trayectoria. Solicitando cancelación...')
                    try:
                        esperar(handle.cancel_goal_async(), 3.0)
                    except (RuntimeError, TimeoutError):
                        print('No fue posible confirmar la cancelación.')
                    objetivo_activo[0] = None
                    continue
                objetivo_activo[0] = None
                if (respuesta.status != GoalStatus.STATUS_SUCCEEDED or
                        respuesta.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    print('El controlador no completó la trayectoria:',
                          respuesta.result.error_code,
                          respuesta.result.error_string)
                    continue

                time.sleep(0.2)
                qfinal = estado_actual()
                if qfinal is None:
                    print('ERROR: no se pudo verificar el estado final en Gazebo.')
                    continue
                Tfinal = fk(qfinal)
                epf, erf = errores_pose(Tdes, Tfinal)
                error_xyz = float(np.linalg.norm(epf))
                error_ori = float(np.linalg.norm(erf))
                error_art = np.abs(qsol - qfinal)
                print('\nq medida [deg]:', np.round(np.rad2deg(qfinal), 3))
                print('Error articular [deg]:', np.round(np.rad2deg(error_art), 4))
                mostrar_pose('POSE TCP/TOOL ALCANZADA:', Tfinal)
                print('Error XYZ final: %.4f mm' % (error_xyz*1000))
                print('Error orientación final: %.4f°' % np.rad2deg(error_ori))
                if (error_xyz < EPS_POS and error_ori < EPS_ANG
                        and np.all(error_art <= TOL_ARTICULAR)):
                    print('Movimiento 6D completado y verificado.\n')
                else:
                    print('AVISO: resultado final fuera de alguna tolerancia.\n')
            except (ValueError, RuntimeError, TimeoutError, np.linalg.LinAlgError) as ex:
                print('ERROR:', ex)
            except (EOFError, KeyboardInterrupt):
                break
    finally:
        if objetivo_activo[0] is not None and rclpy.ok():
            try:
                esperar(objetivo_activo[0].cancel_goal_async(), 3.0)
                print('Cancelación solicitada para la trayectoria pendiente.')
            except Exception:
                print('No se pudo confirmar la cancelación de la trayectoria.')
        cliente.destroy()
        node.destroy_subscription(sub)
        if rclpy.ok():
            rclpy.shutdown()
        hilo.join(timeout=2.0)
        node.destroy_node()


def main():
    ejecutar_gazebo(fkine_hartech, jacobian_6dof, 'tool0')


if __name__ == '__main__':
    main()
