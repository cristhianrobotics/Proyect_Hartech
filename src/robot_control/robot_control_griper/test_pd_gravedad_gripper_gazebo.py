#!/usr/bin/env python3
"""Lab 7: controlador PD + gravedad aplicado COMO TORQUE en Gazebo Classic.

Solo HARTECH con gripper, brazo 6 GDL, apertura de 30 mm (fija en el modelo
Pinocchio). NO publica /joint_states; los recibe del simulador. Requiere
arm_effort_controller y Gazebo con interfaces effort (gazebo_esfuerzo.launch.py).

Si ya se alcanzo el objetivo, sigue publicando torques para SOSTENER el brazo.
Al salir del nodo deja de existir la realimentacion; detener/pausar primero
Gazebo antes de interrumpir esta prueba.
"""
import argparse
import csv
import shutil
import subprocess
import threading
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.modelo_dinamico_gripper import cargar_modelo_con_gripper

NOMBRES = [f'joint_{i}' for i in range(1, 7)]
QHOME = np.deg2rad(np.array([0, -50, 80, 0, -30, 0], dtype=float))
QDESEADA = np.deg2rad(np.array([20, -55, 75, 10, -25, 10], dtype=float))
QMIN = np.deg2rad(np.array([-160, -95, -110, -150, -70, -160], dtype=float))
QMAX = np.deg2rad(np.array([160, 5, 110, 150, 70, 160], dtype=float))
# Límites de esfuerzo del gazebo_simulation.yaml ORIGINAL (NO datos de motor real).
ESFUERZO_MAX = np.array([50., 50., 30., 10., 10., 5.])
FRACCION_ESFUERZO = 0.85
FREC = 50.0
OMEGA_N = 2.6  # Referencia original del Lab 7 (ABA ideal).
# Ajuste conservador SOLO para esfuerzo en Gazebo: no altera el experimento ABA.
ESCALA_KP_GAZEBO = 2.0
ESCALA_KD_GAZEBO = 2.5
ALFA_FILTRO_DQ = 0.30  # 1 = sin filtrado, 0.30 suaviza ruido de velocidad.
TIEMPO_REGISTRO = 60.0  # Registrar también el régimen estacionario.
AMORTIGUAMIENTO = {'sub': 0.35, 'critico': 1.0, 'sobre': 2.0}


class ControlPDGravedad(Node):
    def __init__(self, modo, gazebo_world):
        super().__init__('pd_gravedad_gripper_esfuerzo')
        self.modo = modo
        self.gazebo_world = gazebo_world
        self.fase_arranque = 'espera'
        self.resultado_gravedad = None
        self.hilo_gravedad = None
        self.t_espera = None
        self.ultimo_aviso_arranque = -1.0
        self.model = cargar_modelo_con_gripper(30.0)
        self.data = self.model.createData()
        self.q = None
        self.dq = None
        self.dq_filtrada = None
        self.ultimo_aviso_no_converge = -1.0
        self.marca_ultimo_estado = None
        self.t0 = None
        self.ultimo_log = -1.0
        self.saturaciones = 0
        self.termino_registro = False
        self.aviso_sin_estado = False
        self.ultima_salida = None
        self.finalizado = False

        # Ganancias consistentes con el experimento de Pinocchio y ABA.
        M0 = pin.crba(self.model, self.data, QHOME).copy()
        M0 = np.triu(M0) + np.triu(M0, 1).T
        inertia_diag = np.diag(M0)
        if (not np.all(np.isfinite(inertia_diag))) or np.any(inertia_diag <= 0.0):
            raise ValueError('La matriz de inercia de Pinocchio no es valida.')
        zeta = AMORTIGUAMIENTO[modo]
        self.kp = ESCALA_KP_GAZEBO * inertia_diag * OMEGA_N**2
        self.kd = ESCALA_KD_GAZEBO * 2.0 * zeta * inertia_diag * OMEGA_N
        self.xyz_objetivo = fkine_gripper(QDESEADA)[:3, 3]

        base = Path.home() / 'Hartech_ws' / 'resultados_lab7_gripper'
        base.mkdir(parents=True, exist_ok=True)
        self.ruta_csv = base / f'pd_gripper_gazebo_esfuerzo_{modo}.csv'
        self.archivo = self.ruta_csv.open('w', encoding='utf-8', newline='')
        self.csv = csv.writer(self.archivo)
        self.csv.writerow(['t_s', *[f'q{i}_rad' for i in range(1, 7)],
                           *[f'dq{i}_rad_s' for i in range(1, 7)],
                           *[f'dq_filtrada{i}_rad_s' for i in range(1, 7)],
                           *[f'tau{i}_Nm' for i in range(1, 7)],
                           'error_q_deg', 'error_tcp_mm', 'saturado'])

        self.pub = self.create_publisher(
            Float64MultiArray, '/arm_effort_controller/commands', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.recibir_estados, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/FREC, self.controlar)
        self.get_logger().info('=== PD + GRAVEDAD | TORQUES REALES EN GAZEBO ===')
        self.get_logger().info('Modo: %s | 50 Hz | gripper 30 mm (modelo)' % modo)
        self.get_logger().info('HOME [deg]: %s' % np.rad2deg(QHOME).tolist())
        self.get_logger().info('Destino [deg]: %s' % np.rad2deg(QDESEADA).tolist())
        self.get_logger().info('AJUSTE GAZEBO: Kp x%.2f, Kd x%.2f; filtro de velocidad alpha=%.2f' %
                               (ESCALA_KP_GAZEBO, ESCALA_KD_GAZEBO, ALFA_FILTRO_DQ))
        self.get_logger().info('Kp: %s' % np.round(self.kp, 5))
        self.get_logger().info('Kd: %s' % np.round(self.kd, 5))
        self.get_logger().info('Publicando torques /arm_effort_controller/commands')
        self.get_logger().info('Arranque seguro: mundo 0g -> PD disponible -> gravedad -9.81 -> torques PD.')
        self.get_logger().info('El nodo debe seguir vivo para mantener el brazo. Ctrl+C despues de cerrar/pausar Gazebo.')

    def recibir_estados(self, msg):
        try:
            indices = [msg.name.index(name) for name in NOMBRES]
        except ValueError:
            return
        if len(msg.position) <= max(indices) or len(msg.velocity) <= max(indices):
            return
        q = np.array([msg.position[i] for i in indices], dtype=float)
        dq = np.array([msg.velocity[i] for i in indices], dtype=float)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
            self.get_logger().error('Se recibieron estados no finitos.')
            return
        self.q, self.dq = q, dq
        self.marca_ultimo_estado = self.get_clock().now().nanoseconds / 1e9

    def solicitar_gravedad(self):
        """Cambiar a gravedad normal sin bloquear el bucle ROS de 50 Hz."""
        try:
            if not shutil.which('gz'):
                raise RuntimeError('No se encontro la orden `gz` de Gazebo Classic.')
            resultado = subprocess.run(
                ['gz', 'physics', '-w', self.gazebo_world, '-g', '0,0,-9.81'],
                capture_output=True, text=True, timeout=12, check=False)
            if resultado.returncode:
                raise RuntimeError('gz physics fallo (%d): %s %s' % (
                    resultado.returncode, resultado.stdout.strip(),
                    resultado.stderr.strip()))
            self.resultado_gravedad = (True, 'Gazebo: gravedad normal 0 0 -9.81 activada.')
        except Exception as error:
            self.resultado_gravedad = (False, str(error))

    def preparar_arranque(self, ahora, q, dq):
        """No mandar g(q) mientras el mundo tenga gravedad CERO.

        Enviamos TORQUE CERO en la etapa 0g, esperamos estados estables
        y pasamos a PD+g inmediatamente despues de confirmar la gravedad.
        """
        if self.pub.get_subscription_count() < 1:
            if ahora - self.ultimo_aviso_arranque >= 1.0:
                self.get_logger().info('Esperando suscriptor /arm_effort_controller/commands...')
                self.ultimo_aviso_arranque = ahora
            return False

        if self.fase_arranque == 'espera':
            self.pub.publish(Float64MultiArray(data=[0.0]*6))
            dist_home = np.rad2deg(np.max(np.abs(q-QHOME)))
            vmax = np.max(np.abs(dq))
            # Si no esta en HOME es una simulacion incorrecta o no es el modelo esperado.
            if dist_home > 8.0:
                self.get_logger().error('No esta cerca de HOME en el mundo sin gravedad: '
                                        'max |q-HOME|=%.2f deg. Detener y reiniciar Gazebo.' % dist_home)
                self.finalizado = True
                return False
            if vmax > 0.15:
                if ahora - self.ultimo_aviso_arranque >= 1.0:
                    self.get_logger().info('Esperando que el brazo se estabilice (sin gravedad)...')
                    self.ultimo_aviso_arranque = ahora
                self.t_espera = None
                return False
            if self.t_espera is None:
                self.t_espera = ahora
                return False
            if ahora - self.t_espera < 0.20:
                return False
            self.fase_arranque = 'habilitando'
            self.hilo_gravedad = threading.Thread(target=self.solicitar_gravedad, daemon=True)
            self.hilo_gravedad.start()
            self.get_logger().info('HOME estable. Habilitando gravedad mediante Gazebo...')

        if self.fase_arranque == 'habilitando':
            self.pub.publish(Float64MultiArray(data=[0.0]*6))
            if self.resultado_gravedad is None:
                return False
            correcto, mensaje = self.resultado_gravedad
            if not correcto:
                self.get_logger().error('No se pudo habilitar gravedad: '+mensaje)
                self.get_logger().error('Cerrar esta simulacion y revisar `gz physics -h`.')
                self.finalizado = True
                return False
            self.get_logger().info(mensaje)
            self.fase_arranque = 'activo'
            self.t0 = ahora
            self.ultimo_log = -1.0
            self.get_logger().info('Comenzando PD + compensacion de gravedad.')
            return True

        return self.fase_arranque == 'activo'

    def controlar(self):
        if self.q is None:
            if not self.aviso_sin_estado:
                self.get_logger().info('Esperando /joint_states de Gazebo...')
                self.aviso_sin_estado = True
            return
        ahora = self.get_clock().now().nanoseconds / 1e9
        if self.marca_ultimo_estado is None or ahora - self.marca_ultimo_estado > 0.30:
            # No aplicar torque calculado con un estado obsoleto.
            if not self.finalizado:
                self.get_logger().error('Estado articular obsoleto (>0.30 s). Pausa Gazebo y revisa /joint_states.')
                self.finalizado = True
            return
        if self.finalizado:
            return
        q, dq = self.q.copy(), self.dq.copy()
        if self.fase_arranque != 'activo':
            if not self.preparar_arranque(ahora, q, dq):
                return

        if np.any(q < QMIN - np.deg2rad(2)) or np.any(q > QMAX + np.deg2rad(2)):
            self.get_logger().error('Limite articular excedido. Pausa Gazebo para una parada segura.')
            self.finalizado = True
            return
        if np.any(np.abs(dq) > 1.4):
            self.get_logger().error('Velocidad articular excesiva (>1.4 rad/s). Pausa Gazebo.')
            self.finalizado = True
            return

        # El controlador diferencial del Lab7 utiliza q y dq reales de Gazebo.
        # Filtramos SOLO la velocidad usada en el termino D para reducir el
        # temblor por ruido numerico; el chequeo de seguridad usa dq SIN filtrar.
        if self.dq_filtrada is None:
            self.dq_filtrada = dq.copy()
        else:
            self.dq_filtrada = (ALFA_FILTRO_DQ * dq
                                + (1.0 - ALFA_FILTRO_DQ) * self.dq_filtrada)
        g = pin.computeGeneralizedGravity(self.model, self.data, q).copy()
        # qd constante -> velocidad deseada = 0.
        tau_pd = self.kp * (QDESEADA - q) - self.kd * self.dq_filtrada
        tau = tau_pd + g
        if not np.all(np.isfinite(tau)):
            self.get_logger().error('Torques no finitos. Pausa Gazebo.')
            self.finalizado = True
            return
        lim = FRACCION_ESFUERZO * ESFUERZO_MAX
        saturado = bool(np.any(np.abs(tau) > lim))
        if saturado:
            self.saturaciones += 1
        tau = np.clip(tau, -lim, lim)
        self.pub.publish(Float64MultiArray(data=tau.tolist()))

        error_q = np.linalg.norm(QDESEADA - q)
        xyz = fkine_gripper(q)[:3, 3]
        error_tcp = np.linalg.norm(self.xyz_objetivo - xyz)
        transcurrido = max(0.0, ahora - self.t0)
        if transcurrido <= TIEMPO_REGISTRO:
            self.csv.writerow([f'{transcurrido:.4f}', *q, *dq, *self.dq_filtrada, *tau,
                               np.rad2deg(error_q), 1000*error_tcp, int(saturado)])
        elif not self.termino_registro:
            self.archivo.flush()
            self.termino_registro = True
            self.get_logger().info(f'Registro CSV completado: {self.ruta_csv}')
            self.get_logger().info(f'Eventos de saturacion de esfuerzo: {self.saturaciones}')
            self.get_logger().info('CONTROL CONTINUA ACTIVO para sostener HARTECH; cierra/pausa Gazebo antes de detener.')

        if (transcurrido > 12.0 and (np.rad2deg(error_q) > 3.0 or 1000*error_tcp > 15.0)
                and transcurrido - self.ultimo_aviso_no_converge >= 8.0):
            self.ultimo_aviso_no_converge = transcurrido
            self.get_logger().warn('PD aun lejos del objetivo: revisar q, dq y torques en el CSV; '
                                   'no se considera estabilizado.')

        if transcurrido - self.ultimo_log >= 1.0:
            self.ultimo_log = transcurrido
            self.get_logger().info(f't={transcurrido:5.1f}s | e_q={np.rad2deg(error_q):7.3f}° | '
                                   f'e_TCP={1000*error_tcp:8.3f}mm | '
                                   f'max|tau|={max(abs(tau)):6.3f}Nm | '
                                   f'max|dq|={np.rad2deg(max(abs(dq))):6.2f}deg/s | sat={int(saturado)}')

    def close(self):
        try:
            if not self.archivo.closed:
                self.archivo.flush()
                self.archivo.close()
        except Exception:
            pass
        self.get_logger().info(f'CSV: {self.ruta_csv}')


def main():
    parser = argparse.ArgumentParser(description='PD + gravedad por esfuerzo en Gazebo (Lab 7).')
    parser.add_argument('--modo', choices=list(AMORTIGUAMIENTO), default='critico')
    parser.add_argument('--gazebo-world', default='robot_lab')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = None
    try:
        node = ControlPDGravedad(args.modo, args.gazebo_world)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
