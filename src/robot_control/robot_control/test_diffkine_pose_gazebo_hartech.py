#!/usr/bin/env python3
"""HARTECH sin gripper: control cinemático diferencial de POSE en Gazebo.

Ley del Lab 6 extendida a 6D (tool0):
    twist = [Kp*(pd-p), Ko*log(Rd R.T)]
    qdot = DLS(J6x6, twist)
    qref = q + qdot*dt   (integración Euler)

Cada 10 pasos de 0.02 s se envía una referencia a arm_controller y se
realimenta con las posiciones articulares medidas en /joint_states.
No se publica /joint_states y NO se modifica ningún archivo existente.
El archivo de trayectorias reutiliza las funciones de este programa.
"""
import csv
from pathlib import Path
import time

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof
from robot_control.ikine_pose_6d import errores_pose, rotacion_a_rpy

JOINTS = [f'joint_{i}' for i in range(1, 7)]
DT = 0.02
PASOS_POR_TRAMO = 10                 # 10 x 0.02 = 0.20 segundos
TIEMPO_TRAMO = DT * PASOS_POR_TRAMO
K_POS = 0.9                         # [1/s]
K_ORI = 0.9                         # [1/s]
LONGITUD_ESCALA = 0.30              # [m]
AMORTIGUAMIENTO = 0.015             # DLS para singularidades
FACTOR_VELOCIDAD = 0.35             # 35% de la velocidad configurada en Gazebo
TOL_POS = 0.001                     # [m]
TOL_ORI = np.deg2rad(0.5)           # [rad]
MAX_TRAMOS = 240
TIMEOUT = 20.0
ESTADO_MAX_EDAD = 1.5


def duracion_ros(seg):
    ns = int(round(float(seg) * 1e9))
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def esperar_futuro(node, futuro, timeout):
    fin = time.monotonic() + timeout
    while rclpy.ok() and not futuro.done() and time.monotonic() < fin:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not futuro.done():
        raise TimeoutError('Sin respuesta de arm_controller.')
    if futuro.result() is None:
        raise RuntimeError('arm_controller devolvió una respuesta vacía.')
    return futuro.result()


def leer_limites():
    paquete = Path(get_package_share_directory('robot_description'))
    parametros = yaml.safe_load((paquete / 'config/robot_parameters.yaml').read_text())
    simulacion = yaml.safe_load((paquete / 'config/gazebo_simulation.yaml').read_text())
    qmin = np.array([float(parametros['joints'][j]['lower']) for j in JOINTS])
    qmax = np.array([float(parametros['joints'][j]['upper']) for j in JOINTS])
    vmax = np.array([float(simulacion['joints'][j]['velocity']) for j in JOINTS])
    if (np.any(qmin >= qmax) or np.any(vmax <= 0) or
            not all(np.all(np.isfinite(a)) for a in (qmin, qmax, vmax))):
        raise ValueError('Límites articulares inválidos.')
    return qmin, qmax, vmax


def paso_diferencial(q, Tdes, v_ff, w_ff, qmin, qmax, vmax):
    """Un paso Euler: error XYZ + orientación y pseudoinversa amortiguada 6x6.

    v_ff [m/s] y w_ff [rad/s] están expresadas en ejes base_link.
    """
    T = fkine_hartech(q)
    ep, er = errores_pose(Tdes, T)
    velocidad_deseada = np.r_[v_ff + K_POS*ep, LONGITUD_ESCALA*(w_ff + K_ORI*er)]
    J = jacobian_6dof(q).copy()
    J[3:, :] *= LONGITUD_ESCALA
    qdot = J.T @ np.linalg.solve(J @ J.T + AMORTIGUAMIENTO**2*np.eye(6),
                                  velocidad_deseada)
    if not np.all(np.isfinite(qdot)):
        raise ValueError('Velocidades articulares no finitas.')
    escala = max(1.0, float(np.max(np.abs(qdot) / (FACTOR_VELOCIDAD*vmax))))
    qdot /= escala
    return np.clip(q + qdot*DT, qmin, qmax)


def referencia_fija(t, Tinicio, Tfinal):
    """Referencia constante en el tiempo para control de pose 6D."""
    return Tfinal, np.zeros(3), np.zeros(3)


def controlar_gazebo(referencia, duracion, etiqueta, registro):
    """Bucle realimentado compartido con seguimiento de trayectoria cartesiana.

    referencia(t, Tinicio, Tfinal) -> (Tdes, v_ff, w_ff)
    duracion=0: pose fija; duracion>0: referencia variante en el tiempo.
    """
    qmin, qmax, vmax = leer_limites()
    rclpy.init()
    node = rclpy.create_node(etiqueta, parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    cliente = ActionClient(node, FollowJointTrajectory,
                           '/arm_controller/follow_joint_trajectory')
    estado = {'q': None, 'tiempo': 0.0}
    objetivo_activo = [None]

    def recibir_estado(msg):
        p = dict(zip(msg.name, msg.position))
        if all(j in p for j in JOINTS):
            q = np.array([p[j] for j in JOINTS], dtype=float)
            if np.all(np.isfinite(q)):
                estado['q'] = q
                estado['tiempo'] = time.monotonic()

    sub = node.create_subscription(JointState, '/joint_states', recibir_estado, 10)

    def q_medido():
        if estado['q'] is None or time.monotonic()-estado['tiempo'] > ESTADO_MAX_EDAD:
            return None
        return estado['q'].copy()

    ruta = Path.home() / 'Hartech_ws' / 'resultados_lab6'
    ruta.mkdir(parents=True, exist_ok=True)
    try:
        print(f'\n=== HARTECH sin gripper | {etiqueta} ===')
        print('Referencia: tool0 | Jacobiano 6x6 | Euler dt=0.02 s | DLS')
        print('Ley de control: XYZ + error de orientación (SO3)')
        print('Esperando /arm_controller/follow_joint_trajectory ...')
        if not cliente.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('No está activo arm_controller. Inicia Gazebo.')
        limite = time.monotonic() + TIMEOUT
        while rclpy.ok() and q_medido() is None and time.monotonic() < limite:
            rclpy.spin_once(node, timeout_sec=0.1)
        q0 = q_medido()
        if q0 is None:
            raise TimeoutError('No llegan posiciones recientes de /joint_states.')
        if np.any(q0 < qmin - 1e-3) or np.any(q0 > qmax + 1e-3):
            raise ValueError('La configuración inicial excede límites articulares.')

        Tinicio = fkine_hartech(q0)
        qhome = np.deg2rad([0, -50, 80, 0, -30, 0])
        qotra = np.deg2rad([30, -45, 60, 0, -20, 0])
        T_home, T_otra = fkine_hartech(qhome), fkine_hartech(qotra)
        # Desde HOME, mover a OTRA. Si ya se llegó a OTRA, regresar a HOME.
        distancia_otra = np.linalg.norm(Tinicio[:3, 3] - T_otra[:3, 3])
        Tinicio_ori = np.linalg.norm(errores_pose(T_otra, Tinicio)[1])
        volver_home = distancia_otra < 0.03 and Tinicio_ori < np.deg2rad(10)
        Tfinal = T_home if volver_home else T_otra
        print('q inicial REAL [°]:', np.round(np.rad2deg(q0), 3))
        print('Destino:', 'HOME' if volver_home else 'OTRA')
        print('XYZ deseada [m]:', np.round(Tfinal[:3, 3], 6))
        print('RPY deseada [°]:', np.round(rotacion_a_rpy(Tfinal[:3, :3]), 4))
        if duracion > 0:
            print(f'Trayectoria cartesiana interpolada: {duracion:.1f} s')
        print('Movimiento en Gazebo con realimentación cada 0.20 s.')

        with (ruta / registro).open('w', newline='') as arch:
            writer = csv.writer(arch)
            writer.writerow(['tramo', 'tiempo_referencia_s', 'x_d', 'y_d', 'z_d',
                             'x_real', 'y_real', 'z_real', 'error_xyz_mm',
                             'error_orientacion_deg'])
            error_anterior = float('inf')
            estancado = 0
            completado = False
            for tramo in range(1, MAX_TRAMOS + 1):
                qreal = q_medido()
                if qreal is None:
                    raise RuntimeError('Se perdieron los estados articulares de Gazebo.')
                t = (tramo-1) * TIEMPO_TRAMO
                Tdes, _, _ = referencia(t, Tinicio, Tfinal)
                Treal = fkine_hartech(qreal)
                ep, er = errores_pose(Tdes, Treal)
                pos_mm = 1000*np.linalg.norm(ep)
                ori_grados = np.rad2deg(np.linalg.norm(er))
                writer.writerow([tramo, round(t, 3), *Tdes[:3, 3],
                                 *Treal[:3, 3], pos_mm, ori_grados])
                if tramo == 1 or tramo % 5 == 0 or (t >= duracion and
                                                     pos_mm < 1 and ori_grados < 0.5):
                    print(f'Tramo {tramo:3d} | t={t:5.1f}s | XYZ {pos_mm:7.3f} mm | '
                          f'orientación {ori_grados:6.3f}°')
                if t >= duracion and pos_mm < 1.0 and ori_grados < 0.5:
                    completado = True
                    print('Objetivo alcanzado y verificado con /joint_states.')
                    print('q final REAL [°]:', np.round(np.rad2deg(qreal), 3))
                    print('XYZ REAL [m]:', np.round(Treal[:3, 3], 6))
                    break

                # Sólo aplicar detector de estancamiento después de terminar
                # la referencia temporal; durante la trayectoria el objetivo se mueve.
                if t >= duracion:
                    coste = np.hypot(np.linalg.norm(ep),
                                     LONGITUD_ESCALA*np.linalg.norm(er))
                    if coste < error_anterior - 2e-6:
                        error_anterior, estancado = coste, 0
                    else:
                        estancado += 1
                    if estancado >= 40:
                        print('Detenido: no disminuye el error final. Verifica singularidades o límites.')
                        break

                # Un tramo de 10 pasos Euler (0.2 s), sin resolver IK como salto articular.
                qref = qreal.copy()
                for j in range(PASOS_POR_TRAMO):
                    tiempo_paso = t + j*DT
                    Tstep, v_ff, w_ff = referencia(tiempo_paso, Tinicio, Tfinal)
                    qref = paso_diferencial(qref, Tstep, v_ff, w_ff, qmin, qmax, vmax)
                if np.max(np.abs(qref-qreal)) < 1e-9:
                    print('Sin cambio articular útil; revisa límites y singularidades.')
                    break

                objetivo = FollowJointTrajectory.Goal()
                objetivo.trajectory.joint_names = JOINTS
                punto = JointTrajectoryPoint()
                punto.positions = qref.tolist()
                punto.time_from_start = duracion_ros(TIEMPO_TRAMO)
                objetivo.trajectory.points = [punto]
                objetivo.goal_time_tolerance = duracion_ros(2.0)
                handle = esperar_futuro(node, cliente.send_goal_async(objetivo), TIMEOUT)
                if not handle.accepted:
                    raise RuntimeError(f'arm_controller rechazó tramo {tramo}.')
                objetivo_activo[0] = handle
                try:
                    resultado = esperar_futuro(node, handle.get_result_async(), TIMEOUT)
                except TimeoutError:
                    print('Cancelando trayectoria sin respuesta...')
                    esperar_futuro(node, handle.cancel_goal_async(), 3.0)
                    objetivo_activo[0] = None
                    raise
                objetivo_activo[0] = None
                if (resultado.status != GoalStatus.STATUS_SUCCEEDED or
                        resultado.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    raise RuntimeError(f'Tramo {tramo}: controlador devolvió código '
                                       f'{resultado.result.error_code}: '
                                       f'{resultado.result.error_string}')
            if not completado:
                print('Prueba terminada SIN alcanzar las tolerancias finales.')
        print('Registro guardado en:', ruta / registro)
    except KeyboardInterrupt:
        print('\nMovimiento interrumpido por el usuario.')
    except (RuntimeError, TimeoutError, ValueError, np.linalg.LinAlgError) as exc:
        print('ERROR:', exc)
    finally:
        if objetivo_activo[0] is not None and rclpy.ok():
            try:
                esperar_futuro(node, objetivo_activo[0].cancel_goal_async(), 3.0)
            except Exception:
                print('No se pudo confirmar cancelación de la trayectoria activa.')
        cliente.destroy()
        node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    controlar_gazebo(referencia_fija, duracion=0.0,
                     etiqueta='control_pose_6d_hartech_gazebo',
                     registro='control_pose6d_gazebo.csv')


if __name__ == '__main__':
    main()
