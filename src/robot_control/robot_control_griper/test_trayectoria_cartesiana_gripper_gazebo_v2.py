#!/usr/bin/env python3
"""HARTECH CON GRIPPER - V2: cuadrado XY con los dedos mirando hacia abajo.

Se conserva la ley de control diferencial 6D del archivo EXISTENTE:
  robot_control_griper/test_diffkine_pose_gripper_gazebo.py

Etapas:
  1) Desde HOME, colocar el brazo en la postura de escritura (articulaciones).
  2) Recorrer un cuadrado de 6 x 6 cm en XY, manteniendo Z y R del TCP.
  3) Guardar CSV (controlador original) y gráfica XY deseada/medida.

El eje +Z del TCP apunta hacia -Z de base_link durante el cuadrado.
La "punta del plumon" es conceptual; NO se añade un lápiz ni tinta a Gazebo.
No se modifica ninguna función o nodo anterior.
"""
import csv
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.test_diffkine_pose_gripper_gazebo import (
    JOINTS, ESTADO_MAX_EDAD, TIMEOUT, controlar_gazebo,
    duracion_ros, esperar_futuro, leer_limites,
)
from robot_control.ikine_pose_6d import errores_pose

# Punto de partida seguro para ESTA prueba: HOME con el gripper.
Q_HOME = np.deg2rad([0, -50, 80, 0, -30, 0])

# Postura de escritura calculada con el modelo HARTECH. El +Z del TCP
# apunta verticalmente hacia abajo (-Z de base_link).
Q_ESCRITURA = np.deg2rad([0, -55, 95, 0, 50, 0])

LADO = 0.06                 # 6 cm, cuadrado horizontal
TIEMPO_POR_LADO = 3.0       # 5 s nominales por lado
TIEMPO_TOTAL = 4 * TIEMPO_POR_LADO
DURACION_PREPARACION = 7.0  # s, movimiento articular lento desde HOME

# Para interpretar el dibujo: un plumón virtual sobresale unos centímetros
# desde el TCP. NO existe un objeto físico ni un contacto simulado.
Z_PLANO_VIRTUAL = 0.01  # 1 cm sobre z=0; únicamente referencia conceptual

REGISTRO = 'trayectoria_cuadrado_xy_gripper_v2_gazebo.csv'
ETIQUETA = 'trayectoria_cuadrado_xy_gripper_v2_gazebo'


def acercar_a_postura_escritura():
    """Orden articular previa usando el arm_controller que YA existe.

    Para no arriesgar una entrada a la zona baja desde una postura desconocida,
    solo se permite comenzar cerca de HOME o de la postura de escritura.
    """
    qmin, qmax, vmax = leer_limites()
    if np.any(Q_ESCRITURA < qmin) or np.any(Q_ESCRITURA > qmax):
        raise ValueError('La postura de escritura excede límites articulares.')
    rclpy.init()
    node = rclpy.create_node('preparar_postura_escritura_gripper',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    cliente = ActionClient(node, FollowJointTrajectory,
                           '/arm_controller/follow_joint_trajectory')
    estado = {'q': None, 'tiempo': 0.0}
    goal_activo = [None]

    def callback(msg):
        posiciones = dict(zip(msg.name, msg.position))
        if all(j in posiciones for j in JOINTS):
            q = np.array([posiciones[j] for j in JOINTS], dtype=float)
            if np.all(np.isfinite(q)):
                estado['q'], estado['tiempo'] = q, time.monotonic()

    sub = node.create_subscription(JointState, '/joint_states', callback, 10)
    try:
        print('\n[Etapa 1/2] Preparando gripper para apuntar hacia abajo...')
        if not cliente.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('No está activo /arm_controller/follow_joint_trajectory.')
        fin = time.monotonic() + TIMEOUT
        while rclpy.ok() and estado['q'] is None and time.monotonic() < fin:
            rclpy.spin_once(node, timeout_sec=0.1)
        if estado['q'] is None:
            raise TimeoutError('No hay posiciones de /joint_states.')
        qactual = estado['q'].copy()
        if time.monotonic() - estado['tiempo'] > ESTADO_MAX_EDAD:
            raise RuntimeError('Las posiciones articulares no están actualizadas.')
        print('q inicial REAL [grados]:', np.round(np.rad2deg(qactual), 3))

        if np.max(np.abs(qactual - Q_ESCRITURA)) < np.deg2rad(2.0):
            print('El gripper ya está en la postura de escritura.')
            return
        if np.max(np.abs(qactual - Q_HOME)) > np.deg2rad(12.0):
            raise ValueError('Esta versión solo inicia cerca de HOME. '
                             'Reinicia Gazebo con pose:=home.')
        if np.any(qactual < qmin - 1e-3) or np.any(qactual > qmax + 1e-3):
            raise ValueError('El robot está fuera de límites articulares.')

        # Trayectoria articular de preparación. El cuadrado posterior SI es
        # cartesiano y se gobierna con el Jacobiano y la ley diferencial.
        duracion = max(DURACION_PREPARACION,
                       float(np.max(np.abs(Q_ESCRITURA-qactual)/vmax))*2.0)
        puntos = []
        for fraccion in (0.5, 1.0):
            punto = JointTrajectoryPoint()
            punto.positions = (qactual + fraccion*(Q_ESCRITURA-qactual)).tolist()
            punto.time_from_start = duracion_ros(fraccion*duracion)
            puntos.append(punto)
        objetivo = FollowJointTrajectory.Goal()
        objetivo.trajectory.joint_names = JOINTS
        objetivo.trajectory.points = puntos
        objetivo.goal_time_tolerance = duracion_ros(3.0)
        manejador = esperar_futuro(node, cliente.send_goal_async(objetivo), TIMEOUT)
        if not manejador.accepted:
            raise RuntimeError('arm_controller rechazó la postura de preparación.')
        goal_activo[0] = manejador
        resultado = esperar_futuro(node, manejador.get_result_async(), duracion + TIMEOUT)
        goal_activo[0] = None
        if (resultado.status != GoalStatus.STATUS_SUCCEEDED or
                resultado.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError('No se completó la postura de preparación: '
                               + str(resultado.result.error_string))
        rclpy.spin_once(node, timeout_sec=0.15)
        qmedida = estado['q']
        error_q = np.max(np.abs(np.rad2deg(qmedida-Q_ESCRITURA)))
        T = fkine_gripper(qmedida)
        alineacion = float(np.dot(T[:3, 2], [0., 0., -1.]))
        if error_q > 2.0 or alineacion < np.cos(np.deg2rad(4)):
            raise RuntimeError('No se alcanzó correctamente la postura vertical. '
                               f'Error articular máximo: {error_q:.2f}°.')
        print('Postura alcanzada [grados]:', np.round(np.rad2deg(qmedida), 3))
        print('TCP aproximado [m]:', np.round(T[:3, 3], 4))
        print('Eje +Z TCP en base_link:', np.round(T[:3, 2], 4))
    finally:
        if goal_activo[0] is not None and rclpy.ok():
            try:
                esperar_futuro(node, goal_activo[0].cancel_goal_async(), 3.0)
            except Exception:
                pass
        cliente.destroy()
        node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def trayectoria_cuadrado_v2(t, Tinicio, Tfinal):
    """Cuadrado XY de 6 cm: P0 -> +X -> +Y -> -X -> -Y -> P0.

    Mantiene el TCP con la MISMA orientación vertical hacia abajo y Z fija.
    Conserva la interfaz del controlador diferencial 6D existente:
    (Tdeseada, velocidad_lineal_ff, velocidad_angular_ff).
    """
    if abs(float(t)) < 1e-12:
        Tnom = fkine_gripper(Q_ESCRITURA)
        ep, er = errores_pose(Tnom, Tinicio)
        if np.linalg.norm(ep) > 0.025 or np.linalg.norm(er) > np.deg2rad(8):
            raise ValueError('El TCP no está en la postura de escritura. '
                             'Reinicia Gazebo desde HOME.')
        if np.dot(Tinicio[:3, 2], [0., 0., -1.]) < np.cos(np.deg2rad(8)):
            raise ValueError('El gripper NO está apuntando hacia abajo.')
    p0 = Tinicio[:3, 3]
    vertices = (
        p0,
        p0 + np.array([LADO, 0., 0.]),
        p0 + np.array([LADO, LADO, 0.]),
        p0 + np.array([0., LADO, 0.]),
        p0,
    )
    t = float(np.clip(t, 0.0, TIEMPO_TOTAL))
    lado = min(int(t / TIEMPO_POR_LADO), 3)
    u = (t - lado*TIEMPO_POR_LADO) / TIEMPO_POR_LADO
    s = 3*u*u - 2*u*u*u        # velocidad cero en los vértices
    ds = 6*u*(1-u)/TIEMPO_POR_LADO
    delta = vertices[lado+1] - vertices[lado]
    Td = np.eye(4)
    Td[:3, :3] = Tinicio[:3, :3]  # +Z TCP hacia abajo
    Td[:3, 3] = vertices[lado] + s*delta  # Z constante
    return Td, ds*delta, np.zeros(3)


def graficar_cuadrado(ruta):
    """Gráfica 2D opcional de las posiciones publicadas y medidas en Gazebo."""
    if not ruta.is_file():
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        with ruta.open(newline='') as f:
            filas = list(csv.DictReader(f))
        if not filas:
            return
        xd = np.array([[float(f['x_d']), float(f['y_d'])] for f in filas])
        xr = np.array([[float(f['x_real']), float(f['y_real'])] for f in filas])
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.plot(100*xd[:, 0], 100*xd[:, 1], '--', linewidth=2,
                label='Cuadrado deseado')
        ax.plot(100*xr[:, 0], 100*xr[:, 1], linewidth=1.5,
                label='Trayectoria medida')
        ax.scatter(100*xd[0, 0], 100*xd[0, 1], s=70, marker='o',
                   label='Inicio')
        ax.set_xlabel('X [cm] en base_link')
        ax.set_ylabel('Y [cm] en base_link')
        ax.set_title('HARTECH - Cuadrado XY V2: gripper hacia abajo')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        salida = ruta.with_suffix('.png')
        fig.savefig(salida, dpi=160)
        plt.close(fig)
        print('Gráfica del cuadrado guardada en:', salida)
    except (ValueError, OSError, ImportError, KeyError) as exc:
        print('No se pudo generar gráfica (CSV disponible):', exc)


def main():
    Tnom = fkine_gripper(Q_ESCRITURA)
    print('\n=== HARTECH CON GRIPPER | V2: CUADRADO XY COMO PLUMÓN ===')
    print('Los dedos apuntan hacia ABAJO (eje +Z TCP -> -Z base).')
    print('Postura de escritura [grados]:', np.rad2deg(Q_ESCRITURA))
    print('TCP nominal [m]:', np.round(Tnom[:3, 3], 5))
    print('Cuadrado XY: 6 x 6 cm, partiendo hacia +X y luego +Y.')
    print('Altura TCP aproximadamente 8 cm sobre z=0; Z permanece fija.')
    longitud_virtual = Tnom[2, 3] - Z_PLANO_VIRTUAL
    print(f'Plumón CONCEPTUAL: desde el TCP, {100*longitud_virtual:.1f} cm hacia abajo; '
          f'punta virtual a z={100*Z_PLANO_VIRTUAL:.1f} cm.')
    print('ATENCIÓN: no se agrega plumón físico ni se verifica contacto/colisiones.')
    print('Primero se prepara la postura articular; luego se recorre el cuadrado.')

    acercar_a_postura_escritura()
    print('\n[Etapa 2/2] Recorriendo cuadrado XY con el gripper vertical...')
    controlar_gazebo(trayectoria_cuadrado_v2, duracion=TIEMPO_TOTAL,
                     etiqueta=ETIQUETA, registro=REGISTRO)
    ruta = Path.home() / 'Hartech_ws' / 'resultados_lab6' / REGISTRO
    graficar_cuadrado(ruta)


if __name__ == '__main__':
    main()
