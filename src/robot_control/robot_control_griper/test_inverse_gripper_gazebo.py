#!/usr/bin/env python3
"""Cinemática inversa de posición HARTECH + trayectorias Gazebo Classic/ROS 2 Humble."""
import math
import threading
import time
from pathlib import Path
from datetime import datetime

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')  # No abrir GUI; guardar PNG.
import matplotlib.pyplot as plt
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from robot_control_griper.fkine_gripper import fkine_gripper

JOINTS = [f'joint_{i}' for i in range(1, 7)]
EPS_POS = 0.001                  # m: error máximo de IK
TOL_ART = np.deg2rad(1.0)       # rad: error máximo articular al final
FRACCION_VELOCIDAD = 0.35      # porcentaje de velocidad máxima de simulación
MAX_ITER = 250
TIMEOUT = 30.0


def fk_pos(q):
    return fkine_gripper(q)[:3, 3]


def jacobiano_numerico(q, h=1e-5):
    """Jacobiano de posición 3x6 con diferencias centrales."""
    J = np.empty((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        J[:, i] = (fk_pos(q + dq) - fk_pos(q - dq)) / (2.0 * h)
    return J


def resolver_desde_semilla(xdes, semilla, qmin, qmax):
    """Newton amortiguado con límites en CADA paso, backtracking y paso máximo."""
    q = np.clip(np.asarray(semilla, dtype=float).copy(), qmin, qmax)
    historial = []
    mejor_error = float('inf')
    estancado = 0
    for _ in range(MAX_ITER):
        e = xdes - fk_pos(q)
        err = float(np.linalg.norm(e))
        historial.append(err)
        if err < EPS_POS:
            return q, historial, True
        if err < mejor_error - 1e-8:
            mejor_error, estancado = err, 0
        else:
            estancado += 1
            if estancado >= 25:
                break

        J = jacobiano_numerico(q)
        # Amortiguamiento dinámico: limita las soluciones desmesuradas
        lam = 0.015 if err < 0.05 else 0.04
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(3), e)
        except np.linalg.LinAlgError:
            break
        if not np.all(np.isfinite(dq)):
            break
        magnitud = np.linalg.norm(dq)
        if magnitud > 0.2:
            dq *= 0.2 / magnitud

        # Solo aceptar mejoras; nunca exceder límites articulares
        aceptado = False
        for factor in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            candidato = np.clip(q + factor * dq, qmin, qmax)
            if np.linalg.norm(xdes - fk_pos(candidato)) < err - 1e-10:
                q = candidato
                aceptado = True
                break
        if not aceptado:
            break
    historial.append(float(np.linalg.norm(xdes - fk_pos(q))))
    return q, historial, historial[-1] < EPS_POS


def resolver_ik(xdes, qactual, qmin, qmax, qhome):
    """Probar semillas válidas y elegir la solución de menor cambio articular."""
    qvertical = np.deg2rad([0, -90, 0, 0, 0, 0])
    semillas = [qactual.copy(), qhome.copy(), np.zeros(6), qvertical]
    # Semillas adicionales sistemáticas, sin aleatoriedad entre ejecuciones
    for delta in (-0.6, 0.6):
        for i in (0, 1, 2, 4):
            s = qactual.copy()
            s[i] += delta
            semillas.append(s)
    candidatos = []
    mejor_fallido = None
    for semilla in semillas:
        qsol, ee, ok = resolver_desde_semilla(xdes, semilla, qmin, qmax)
        err = float(np.linalg.norm(xdes - fk_pos(qsol)))
        if ok:
            # Desempate suave: preferir soluciones cercanas al estado real
            distancia = float(np.linalg.norm(qsol - qactual))
            candidatos.append((distancia, err, qsol, ee))
        elif mejor_fallido is None or err < mejor_fallido[0]:
            mejor_fallido = (err, ee)
    if not candidatos:
        return None, (mejor_fallido[1] if mejor_fallido else []), False
    candidatos.sort(key=lambda item: (item[0], item[1]))
    _, _, qsol, ee = candidatos[0]
    return qsol, ee, True


def graficar_error(ee):
    nombre = 'error_ikine_gripper_gazebo_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.png'
    plt.figure()
    plt.plot(ee, 'b.-')
    plt.title('Convergencia IK HARTECH TCP - Gazebo')
    plt.xlabel('Iteración')
    plt.ylabel('Norma del error [m]')
    plt.grid(True)
    plt.savefig(nombre, dpi=200, bbox_inches='tight')
    plt.close()
    print('Gráfica guardada en:', nombre)


def duration_msg(seg):
    ns = round(float(seg) * 1e9)
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def esperar(futuro, segundos):
    limite = time.monotonic() + segundos
    while rclpy.ok() and not futuro.done():
        if time.monotonic() >= limite:
            raise TimeoutError('Se agotó el tiempo de espera de Gazebo.')
        time.sleep(0.05)
    if not futuro.done() or futuro.result() is None:
        raise RuntimeError('ROS 2 interrumpido o sin respuesta.')
    return futuro.result()


def main():
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
        raise ValueError('Límites o velocidades de configuración inválidos.')

    rclpy.init()
    node = rclpy.create_node('InverseKinematicsGripperGazebo',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, FollowJointTrajectory,
                          '/arm_controller/follow_joint_trajectory')
    estados = {}
    lock = threading.Lock()
    ultimo = [0.0]
    objetivo_activo = [None]

    def recibir(msg):
        with lock:
            for j, p in zip(msg.name, msg.position):
                estados[j] = p
            if all(j in estados for j in JOINTS):
                ultimo[0] = time.monotonic()

    sub = node.create_subscription(JointState, '/joint_states', recibir, 10)
    hilo = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    hilo.start()

    def estado_actual():
        with lock:
            if not all(j in estados for j in JOINTS):
                return None
            q = np.array([estados[j] for j in JOINTS])
            antiguedad = time.monotonic() - ultimo[0]
        if not np.all(np.isfinite(q)) or antiguedad > 3.0:
            return None
        return q

    try:
        print('Esperando /arm_controller/follow_joint_trajectory ...')
        if not client.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('No está disponible arm_controller. Inicia Gazebo.')
        limite = time.monotonic() + TIMEOUT
        while rclpy.ok() and estado_actual() is None and time.monotonic() < limite:
            time.sleep(0.1)
        if estado_actual() is None:
            raise TimeoutError('No se recibieron /joint_states válidos de Gazebo.')

        print('\n=== CINEMÁTICA INVERSA HARTECH - TCP GRIPPER - GAZEBO ===')
        print('Ingresa x y z DEL TCP en METROS. Ejemplo: 0 0 0.821')
        print("Escribe 'salir' para terminar.\n")
        while rclpy.ok():
            try:
                entrada = input('xdes[m] > ').strip()
                if entrada.lower() in ('salir', 'exit', 'quit'):
                    break
                valores = [float(v) for v in entrada.replace(',', ' ').split()]
                if len(valores) != 3 or not all(math.isfinite(v) for v in valores):
                    print('Ingresa tres números finitos: x y z.')
                    continue
                xdes = np.array(valores)
                qactual = estado_actual()
                if qactual is None:
                    print('Sin información articular reciente de Gazebo.')
                    continue

                print('Semilla real [deg]:', np.round(np.rad2deg(qactual), 2))
                print('Buscando soluciones con límites articulares...')
                qsol, ee, convergio = resolver_ik(xdes, qactual, qmin, qmax, qhome)
                if not convergio:
                    print('No se encontró solución válida dentro de 1 mm.')
                    if ee:
                        print('Mejor error obtenido [m]:', round(ee[-1], 6))
                    continue
                if not np.all((qsol >= qmin) & (qsol <= qmax)):
                    print('Solución fuera de límites. Se rechaza.')
                    continue
                graficar_error(ee)
                print('Posición TCP objetivo [m]:', np.round(xdes, 4))
                print('Posición TCP calculada [m]:', np.round(fk_pos(qsol), 4))
                print('Error IK [m]:', round(np.linalg.norm(xdes - fk_pos(qsol)), 6))
                print('q solución [deg]:', np.round(np.rad2deg(qsol), 2))

                if np.all(np.abs(qsol - qactual) < TOL_ART):
                    print('Articulaciones ya están dentro de 1° de la solución.')
                    continue

                # Tiempo automático con velocidad reducida de simulación
                delta = np.abs(qsol - qactual)
                duracion = max(0.6, 0.1 + 1.2 * 1.5 * np.max(delta / (
                    FRACCION_VELOCIDAD * velocidades)))

                objetivo = FollowJointTrajectory.Goal()
                objetivo.trajectory.joint_names = JOINTS
                inicio = JointTrajectoryPoint()
                inicio.positions = qactual.tolist()
                inicio.velocities = [0.0] * 6
                inicio.time_from_start = duration_msg(0.1)
                final = JointTrajectoryPoint()
                final.positions = qsol.tolist()
                final.velocities = [0.0] * 6
                final.time_from_start = duration_msg(duracion)
                objetivo.trajectory.points = [inicio, final]
                objetivo.goal_time_tolerance = duration_msg(3.0)

                print(f'Enviando trayectoria (tiempo calculado: {duracion:.2f} s)...')
                handle = esperar(client.send_goal_async(objetivo), TIMEOUT)
                if not handle.accepted:
                    print('Gazebo rechazó la trayectoria.')
                    continue
                objetivo_activo[0] = handle
                try:
                    respuesta = esperar(handle.get_result_async(), duracion + TIMEOUT)
                except TimeoutError:
                    print('Trayectoria sin respuesta: solicitando cancelación...')
                    try:
                        esperar(handle.cancel_goal_async(), 3.0)
                    except (TimeoutError, RuntimeError):
                        print('No se pudo confirmar la cancelación.')
                    objetivo_activo[0] = None
                    continue
                objetivo_activo[0] = None
                if (respuesta.status != GoalStatus.STATUS_SUCCEEDED
                    or respuesta.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    print('Fallo de controlador:', respuesta.result.error_code,
                          respuesta.result.error_string)
                    continue
                time.sleep(0.2)
                qfinal = estado_actual()
                if qfinal is None:
                    print('No se puede verificar la postura final (sin estados nuevos).')
                    continue
                error_art = np.rad2deg(np.abs(qsol - qfinal))
                error_xyz = np.linalg.norm(xdes - fk_pos(qfinal))
                print('q medida [deg]:', np.round(np.rad2deg(qfinal), 2))
                print('Error articular [deg]:', np.round(error_art, 4))
                print('Posición TCP alcanzada [m]:', np.round(fk_pos(qfinal), 4))
                print('Error XYZ final [m]:', round(error_xyz, 6))
                if np.all(error_art <= np.rad2deg(TOL_ART)) and error_xyz < EPS_POS:
                    print('Movimiento completado y verificado.\n')
                else:
                    print('AVISO: el objetivo no quedó dentro de todas las tolerancias.\n')
            except (ValueError, RuntimeError, TimeoutError, np.linalg.LinAlgError) as ex:
                print('ERROR:', ex)
            except KeyboardInterrupt:
                break
    finally:
        if objetivo_activo[0] is not None and rclpy.ok():
            try:
                esperar(objetivo_activo[0].cancel_goal_async(), 3.0)
                print('Se solicitó cancelar la trayectoria pendiente.')
            except Exception:
                print('No se pudo confirmar la cancelación de la trayectoria.')
        client.destroy()
        node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        hilo.join(timeout=2.0)


if __name__ == '__main__':
    main()
