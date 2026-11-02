#!/usr/bin/env python3
"""Lab 6 adaptado a HARTECH SIN GRIPPER: control diferencial XYZ en Gazebo.

Reutiliza FK y Jacobiano existentes. Ley del laboratorio:
    qdot = pinv(Jpos(q)) @ (k * (xd - x(q)))
    q_next = q + qdot * dt

Integracion Euler a 50 Hz (dt=0.02 s). Para el controlador de trayectorias
se agrupan 10 pasos (0.20 s) y se REALIMENTA el estado articular MEDIDO
por Gazebo al principio de cada tramo. No se usa KinPy ni IK de pose.
"""
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

JOINTS = [f'joint_{i}' for i in range(1, 7)]
XD = np.array([0.341858, 0.197372, 0.350692], dtype=float)
K = 2 # Ganancia proporcional del control cinemático diferencial.
DT = 0.02                   # Paso de Euler, como el laboratorio: 50 Hz
PASOS_POR_TRAMO = 10        # 10*0.02 = 0.20 s; 5 actualizaciones/seg en Gazebo
TOLERANCIA = 0.001          # 1 mm
MAX_TRAMOS = 120
FACTOR_VELOCIDAD = 0.35     # No usar toda la velocidad nominal de simulacion
ESTADO_MAX_EDAD = 1.5       # Segundos de tiempo de pared
TIMEOUT = 20.0


def duracion_ros(seg):
    ns = int(round(float(seg) * 1e9))
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def esperar_futuro(node, futuro, timeout):
    """Procesa callbacks /joint_states mientras espera una respuesta del action server."""
    limite = time.monotonic() + timeout
    while rclpy.ok() and not futuro.done() and time.monotonic() < limite:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not futuro.done():
        raise TimeoutError('Sin respuesta del controlador de Gazebo.')
    respuesta = futuro.result()
    if respuesta is None:
        raise RuntimeError('El controlador devolvio una respuesta vacia.')
    return respuesta


def siguiente_referencia(q_inicial, xd, qmin, qmax, velocidad_max):
    """Repite la misma ley del Lab 6 durante 10 pasos Euler de 20 ms.

    La medicion real q_inicial se actualiza DESPUES de cada trayectoria corta.
    No existe solucion de IK previa ni se predefine una trayectoria completa.
    """
    q = np.asarray(q_inicial, dtype=float).copy()
    for _ in range(PASOS_POR_TRAMO):
        x = fkine_hartech(q)[:3, 3]
        e = xd - x
        if np.linalg.norm(e) < TOLERANCIA:
            break
        J = jacobian_6dof(q)[:3, :]    # 3x6, exactamente como Lab 6
        qdot = np.linalg.pinv(J) @ (K * e)
        if not np.all(np.isfinite(qdot)):
            raise ValueError('Velocidades articulares no finitas.')
        # Proteccion para el movimiento en Gazebo; no altera la ley de control.
        escala = max(1.0, float(np.max(np.abs(qdot) / velocidad_max)))
        qdot = qdot / escala
        q = np.clip(q + qdot * DT, qmin, qmax)
    return q


def main():
    paquete = Path(get_package_share_directory('robot_description'))
    parametros = yaml.safe_load((paquete / 'config/robot_parameters.yaml').read_text())
    simulacion = yaml.safe_load((paquete / 'config/gazebo_simulation.yaml').read_text())
    qmin = np.array([float(parametros['joints'][j]['lower']) for j in JOINTS])
    qmax = np.array([float(parametros['joints'][j]['upper']) for j in JOINTS])
    velocidades = np.array([float(simulacion['joints'][j]['velocity']) for j in JOINTS])
    if (np.any(qmin >= qmax) or np.any(velocidades <= 0) or
            not np.all(np.isfinite(qmin)) or not np.all(np.isfinite(qmax)) or
            not np.all(np.isfinite(velocidades))):
        raise ValueError('Limites o velocidades no validos en robot_description/config.')

    rclpy.init()
    node = rclpy.create_node('test_diffkine_gazebo_hartech',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, FollowJointTrajectory,
                          '/arm_controller/follow_joint_trajectory')
    estado = {'q': None, 'tiempo': 0.0}
    goal_activo = [None]

    def recibir_estado(msg):
        posiciones = dict(zip(msg.name, msg.position))
        if all(j in posiciones for j in JOINTS):
            q = np.array([posiciones[j] for j in JOINTS], dtype=float)
            if np.all(np.isfinite(q)):
                estado['q'] = q
                estado['tiempo'] = time.monotonic()

    sub = node.create_subscription(JointState, '/joint_states', recibir_estado, 10)

    def q_medido():
        if estado['q'] is None:
            return None
        if time.monotonic() - estado['tiempo'] > ESTADO_MAX_EDAD:
            return None
        return estado['q'].copy()

    ruta = Path.home() / 'Hartech_ws' / 'resultados_lab6'
    ruta.mkdir(parents=True, exist_ok=True)

    try:
        print('\n=== HARTECH SIN GRIPPER | Control cinemático diferencial XYZ en GAZEBO ===')
        print('Ley Lab 6: qdot = pinv(Jpos) @ (k * (xd-x)); Euler')
        print('XYZ deseada [m]:', XD, '| k:', K, '| dt:', DT)
        print('Tramos de 0.20 s con realimentación desde /joint_states.')
        print('No se publica /joint_states: se utiliza el controlador de Gazebo.')
        print('Esperando /arm_controller/follow_joint_trajectory ...')
        if not client.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('No hay arm_controller. Comprueba que Gazebo esté abierto.')
        limite = time.monotonic() + TIMEOUT
        while rclpy.ok() and q_medido() is None and time.monotonic() < limite:
            rclpy.spin_once(node, timeout_sec=0.10)
        q0 = q_medido()
        if q0 is None:
            raise TimeoutError('No llegan posiciones /joint_states desde Gazebo.')
        if np.any(q0 < qmin - 1e-4) or np.any(q0 > qmax + 1e-4):
            raise ValueError('La configuración inicial medida excede los límites articulares.')
        print('q inicial REAL [deg]:', np.round(np.rad2deg(q0), 3))
        print('Error XYZ inicial [mm]:', round(1000 * np.linalg.norm(
            XD - fkine_hartech(q0)[:3, 3]), 4))

        with (ruta / 'xcurrent_gazebo.txt').open('w') as fxc, \
             (ruta / 'xdesired_gazebo.txt').open('w') as fxd, \
             (ruta / 'q_gazebo.txt').open('w') as fq:
            error_previo = float('inf')
            sin_mejora = 0
            for tramo in range(1, MAX_TRAMOS + 1):
                qreal = q_medido()
                if qreal is None:
                    raise RuntimeError('Datos articulares ausentes o desactualizados.')
                xreal = fkine_hartech(qreal)[:3, 3]
                error = float(np.linalg.norm(XD - xreal))
                for archivo, vector in ((fxc, xreal), (fxd, XD), (fq, qreal)):
                    archivo.write(' '.join(map(str, vector)) + '\n')

                if tramo == 1 or tramo % 5 == 0 or error < TOLERANCIA:
                    print(f'Tramo {tramo:3d} | Error XYZ REAL: {error*1000:.4f} mm')
                if error < TOLERANCIA:
                    print('Objetivo alcanzado en Gazebo.')
                    print('q final medida [deg]:', np.round(np.rad2deg(qreal), 3))
                    print('XYZ medida [m]:', np.round(xreal, 6))
                    break

                if error < error_previo - 0.00002:
                    sin_mejora = 0
                    error_previo = error
                else:
                    sin_mejora += 1
                    if sin_mejora >= 20:
                        print('Detenido: el error no está disminuyendo. Revisa límites o colisiones.')
                        break

                qref = siguiente_referencia(qreal, XD, qmin, qmax,
                                            FACTOR_VELOCIDAD * velocidades)
                desplazamiento = float(np.max(np.abs(qref - qreal)))
                if desplazamiento < 1e-8:
                    print('Detenido: el cálculo no produce un cambio articular útil.')
                    break

                objetivo = FollowJointTrajectory.Goal()
                objetivo.trajectory.joint_names = JOINTS
                punto = JointTrajectoryPoint()
                punto.positions = qref.tolist()
                punto.time_from_start = duracion_ros(DT * PASOS_POR_TRAMO)
                objetivo.trajectory.points = [punto]
                objetivo.goal_time_tolerance = duracion_ros(2.0)

                handle = esperar_futuro(node, client.send_goal_async(objetivo), TIMEOUT)
                if not handle.accepted:
                    raise RuntimeError(f'Trayectoria rechazada en tramo {tramo}.')
                goal_activo[0] = handle
                try:
                    respuesta = esperar_futuro(node, handle.get_result_async(), TIMEOUT)
                except TimeoutError:
                    print('Cancelando trayectoria sin respuesta...')
                    esperar_futuro(node, handle.cancel_goal_async(), 3.0)
                    goal_activo[0] = None
                    raise
                goal_activo[0] = None
                if (respuesta.status != GoalStatus.STATUS_SUCCEEDED or
                        respuesta.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    raise RuntimeError('El controlador falló en tramo %d: código=%s, %s' % (
                        tramo, respuesta.result.error_code,
                        respuesta.result.error_string))

            else:
                print('Máximo de tramos alcanzado sin cumplir 1 mm.')
            print('Registros Gazebo guardados en:', ruta)
    except KeyboardInterrupt:
        print('\nEjecución detenida por el usuario.')
    except (ValueError, RuntimeError, TimeoutError, np.linalg.LinAlgError) as exc:
        print('ERROR:', exc)
    finally:
        if goal_activo[0] is not None and rclpy.ok():
            try:
                esperar_futuro(node, goal_activo[0].cancel_goal_async(), 3.0)
            except Exception:
                print('No fue posible confirmar la cancelación del objetivo activo.')
        client.destroy()
        node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
