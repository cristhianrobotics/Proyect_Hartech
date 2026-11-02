#!/usr/bin/env python3
"""Lab 7 HARTECH con gripper: regulación PD + gravedad y dinámica directa ABA.

Adaptación de control_pdg.py del UR5. Simulación del modelo dinámico en Python,
NO envía torques a Gazebo. Opcionalmente publica articulaciones para visualizar RViz.
El gripper forma parte de las masas e inercias; su apertura permanece FIJA.
"""
import argparse
import csv
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.modelo_dinamico_gripper import cargar_modelo_con_gripper

# Configuración del experimento: brazo de 6 GDL + gripper de apertura fija.
Q_INICIAL_DEG = [0, -50, 80, 0, -30, 0]
Q_DESEADA_DEG = [20, -55, 75, 10, -25, 10]
Q_MIN_DEG = [-160, -95, -110, -150, -70, -160]
Q_MAX_DEG = [160, 5, 110, 150, 70, 160]
DT = 0.02  # s (50 Hz)
TMAX = 16.0  # s
FRECUENCIA_NATURAL = 2.6  # rad/s, punto de partida para el ajuste de ganancias
# Estos son niveles NOMINALES de amortiguamiento por articulación; el sistema está acoplado.
AMORTIGUAMIENTO = {'sub': 0.35, 'critico': 1.0, 'sobre': 2.0}
NOMBRES_BRAZO = [f'joint_{i}' for i in range(1, 7)]


def ruta_resultados():
    ruta = Path.home() / 'Hartech_ws' / 'resultados_lab7_gripper'
    ruta.mkdir(parents=True, exist_ok=True)
    return ruta


def simular(modelo, modo='critico', apertura_mm=30.0, visualizar=False):
    """Simula PD+g con ABA. q y dq siempre pertenecen a los seis ejes del brazo."""
    if modo not in AMORTIGUAMIENTO:
        raise ValueError(f'Modo desconocido: {modo}')
    if visualizar and not (0 <= apertura_mm <= 50):
        raise ValueError('La apertura del gripper debe estar entre 0 y 50 mm.')

    datos = modelo.createData()
    q = np.deg2rad(np.array(Q_INICIAL_DEG, dtype=float))
    qd = np.deg2rad(np.array(Q_DESEADA_DEG, dtype=float))
    dq = np.zeros(6)
    qmin, qmax = np.deg2rad(Q_MIN_DEG), np.deg2rad(Q_MAX_DEG)
    xdes = fkine_gripper(qd)[:3, 3]

    # Ganancias diagonales como en el Lab 7; escaladas con la inercia en HOME.
    # zeta=1 NO garantiza amortiguamiento crítico exacto para las 6 articulaciones acopladas.
    M0 = pin.crba(modelo, datos, q).copy()
    M0 = np.triu(M0) + np.triu(M0, 1).T
    inercia_diag = np.diag(M0)
    if not np.all(inercia_diag > 0):
        raise RuntimeError('El modelo devolvió una inercia articular no positiva.')
    zeta = AMORTIGUAMIENTO[modo]
    kp_vec = inercia_diag * FRECUENCIA_NATURAL**2
    kd_vec = 2.0 * zeta * inercia_diag * FRECUENCIA_NATURAL
    Kp, Kd = np.diag(kp_vec), np.diag(kd_vec)
    print(f'\n=== HARTECH + GRIPPER | PD + GRAVEDAD | {modo.upper()} ===')
    print(f'Apertura fija: {apertura_mm:g} mm | dt={DT:.3f}s | tmax={TMAX:.1f}s')
    print('q inicial [°]:', Q_INICIAL_DEG)
    print('q deseada [°]:', Q_DESEADA_DEG)
    print('Kp diagonal:', np.round(kp_vec, 5))
    print('Kd diagonal:', np.round(kd_vec, 5))
    print('Nota: los modos son niveles aproximados de amortiguamiento, no clases exactas.')

    nodo = publicador = None
    if visualizar:
        import rclpy
        from sensor_msgs.msg import JointState
        rclpy.init()
        nodo = rclpy.create_node('pd_gravedad_gripper_rviz')
        publicador = nodo.create_publisher(JointState, '/joint_states', 10)

    salida = ruta_resultados() / f'pd_gripper_{modo}.csv'
    campos = ['t'] + [f'q{i}' for i in range(1, 7)] + [f'dq{i}' for i in range(1, 7)] + [
        'x', 'y', 'z', 'xd', 'yd', 'zd', 'error_q_rad', 'error_xyz_m', 'tau_max_Nm']
    max_error = 0.0
    try:
        with salida.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(campos)
            nsteps = int(np.ceil(TMAX / DT))
            for paso in range(nsteps + 1):
                t = paso * DT
                e = qd - q
                de = -dq  # qd constante -> dq_des = 0
                # Ley de control original del Lab 7: u = Kp*e + Kd*de + g(q)
                g = pin.computeGeneralizedGravity(modelo, datos, q).copy()
                tau = Kp @ e + Kd @ de + g
                x = fkine_gripper(q)[:3, 3]
                error_q = float(np.linalg.norm(e))
                error_xyz = float(np.linalg.norm(xdes - x))
                max_error = max(max_error, error_xyz)
                writer.writerow([t, *q, *dq, *x, *xdes, error_q, error_xyz,
                                 float(np.max(np.abs(tau)))])
                if paso % 50 == 0 or paso == nsteps:
                    print(f't={t:5.2f}s | error articular={np.rad2deg(error_q):7.3f}° '
                          f'| error TCP={1000 * error_xyz:8.3f} mm')
                if publicador is not None:
                    from sensor_msgs.msg import JointState
                    msg = JointState()
                    msg.header.stamp = nodo.get_clock().now().to_msg()
                    msg.name = NOMBRES_BRAZO + ['hartech_left_finger_joint']
                    msg.position = q.tolist() + [apertura_mm / 2000.0]
                    publicador.publish(msg)
                if paso == nsteps:
                    break
                ddq = pin.aba(modelo, datos, q, dq, tau).copy()
                if not np.all(np.isfinite(ddq)):
                    raise RuntimeError('Aceleración no finita; revisar modelo o ganancias.')
                # Euler semiimplícito: primer velocidad, luego posición.
                dq = dq + DT * ddq
                q = q + DT * dq
                if np.any(q < qmin - 1e-4) or np.any(q > qmax + 1e-4):
                    raise RuntimeError('Se superó un límite articular; reducir ganancias/objetivo.')
                if visualizar:
                    time.sleep(DT)
    finally:
        if nodo is not None:
            nodo.destroy_node()
            import rclpy
            rclpy.shutdown()
    print('\nq final [°]:', np.round(np.rad2deg(q), 3))
    print(f'Error final de TCP: {1000 * error_xyz:.3f} mm')
    print(f'Archivo CSV: {salida}')
    return salida


def main():
    parser = argparse.ArgumentParser(description='HARTECH con gripper: PD+g + ABA (Lab 7).')
    parser.add_argument('--modo', choices=[*AMORTIGUAMIENTO, 'todos'], default='critico',
                        help='Nivel de amortiguamiento (sub, critico, sobre, todos).')
    parser.add_argument('--mm', type=float, default=30.0, help='Apertura fija gripper (0..50 mm).')
    parser.add_argument('--rviz', action='store_true', help='Publica /joint_states para visualización en RViz.')
    args = parser.parse_args()
    if args.modo == 'todos' and args.rviz:
        parser.error('--rviz solo funciona con un modo a la vez.')
    modelo = cargar_modelo_con_gripper(args.mm)
    modos = AMORTIGUAMIENTO if args.modo == 'todos' else [args.modo]
    for modo in modos:
        simular(modelo, modo=modo, apertura_mm=args.mm, visualizar=args.rviz)


if __name__ == '__main__':
    main()
