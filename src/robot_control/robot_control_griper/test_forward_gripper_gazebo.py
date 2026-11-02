#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import time
import math
import yaml

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from robot_control_griper.fkine_gripper import fkine_gripper

# Joint names - HARTECH
jnames = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Parámetros del controlador
TIEMPO_MINIMO = 0.5
FACTOR_SEGURIDAD = 1.2
TOLERANCIA_ARTICULAR = np.deg2rad(1.0)
TIMEOUT_CONEXION = 30.0
TIMEOUT_RESULTADO = 15.0

def duracion_ros(segundos):
  """Convertir segundos a Duration de ROS 2."""
  ns = round(segundos * 1e9)
  return Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)

def esperar_futuro(futuro, timeout):
  """Esperar una respuesta ROS 2 con tiempo máximo."""
  inicio = time.monotonic()

  while rclpy.ok() and not futuro.done():
    if time.monotonic() - inicio > timeout:
      raise TimeoutError("Tiempo de espera agotado.")
    time.sleep(0.05)

  if not futuro.done():
    raise RuntimeError("La comunicación ROS 2 fue interrumpida.")

  resultado = futuro.result()
  if resultado is None:
    raise RuntimeError("No se recibió una respuesta válida.")

  return resultado

def main():
  rclpy.init()

  node = rclpy.create_node(
    'ForwardKinematicsGripperGazebo',
    parameter_overrides=[Parameter('use_sim_time', value=True)]
  )

  # Cliente del controlador de Gazebo
  client = ActionClient(
    node,
    FollowJointTrajectory,
    '/arm_controller/follow_joint_trajectory'
  )

  # Leer parámetros del robot HARTECH
  package = Path(get_package_share_directory('robot_description'))

  config = yaml.safe_load(
    (package / 'config/robot_parameters.yaml').read_text()
  )

  simulation = yaml.safe_load(
    (package / 'config/gazebo_simulation.yaml').read_text()
  )

  # Almacenar posiciones articulares actuales
  estado = {}
  estado_lock = threading.Lock()
  ultimo_estado = [0.0]

  def recibir_estado(msg):
    with estado_lock:
      for nombre, posicion in zip(msg.name, msg.position):
        estado[nombre] = posicion

      if all(joint in estado for joint in jnames):
        ultimo_estado[0] = time.monotonic()

  # Suscriptor de estados articulares de Gazebo
  sub = node.create_subscription(
    JointState, '/joint_states', recibir_estado, 10
  )

  # Hilo de comunicación ROS 2
  thread = threading.Thread(
    target=rclpy.spin,
    args=(node,),
    daemon=True
  )
  thread.start()

  try:
    print("\nEsperando controlador de Gazebo...")

    if not client.wait_for_server(timeout_sec=TIMEOUT_CONEXION):
      raise RuntimeError("No se encontró el controlador arm_controller.")

    print("Controlador conectado correctamente.")
    print("Esperando estados articulares...")

    inicio = time.monotonic()

    while rclpy.ok():
      with estado_lock:
        disponible = all(joint in estado for joint in jnames)

      if disponible:
        break

      if time.monotonic() - inicio > TIMEOUT_CONEXION:
        raise TimeoutError("No se reciben estados articulares de Gazebo.")

      time.sleep(0.1)

    if not rclpy.ok():
      raise RuntimeError("ROS 2 se detuvo antes de recibir estados.")

    # Obtener configuración inicial real de Gazebo
    with estado_lock:
      q = np.array([estado[joint] for joint in jnames])

    T = fkine_gripper(q)

    print("\nConfiguración inicial [deg]:")
    print(np.round(np.rad2deg(q), 2))

    print("\nMatriz T TCP inicial:")
    print(np.round(T, 3))

    print("\n=== CINEMATICA DIRECTA HARTECH - TCP GRIPPER - GAZEBO ===")
    print("Ingresa 6 articulaciones en GRADOS separadas por espacio.")
    print("Ejemplo: 0 -50 80 0 -30 0")
    print("Escribe 'salir' para terminar.\n")

    # Lectura de articulaciones desde teclado
    while rclpy.ok():
      try:
        entrada = input("q[deg] > ").strip()

        if entrada.lower() in ['salir', 'exit', 'quit']:
          break

        valores = [float(v) for v in entrada.replace(',', ' ').split()]

        if len(valores) != 6 or not all(math.isfinite(v) for v in valores):
          print("Debes ingresar exactamente 6 valores numéricos.")
          continue

        # Convertir grados a radianes
        q_nuevo = np.deg2rad(np.array(valores))

        # Verificar límites articulares
        valido = True

        for joint, valor in zip(jnames, q_nuevo):
          limite = config['joints'][joint]
          inferior = float(limite['lower'])
          superior = float(limite['upper'])

          if not inferior <= valor <= superior:
            print(f"ERROR: {joint} fuera de sus límites articulares.")
            valido = False

        if not valido:
          continue

        # Obtener posiciones actuales del robot
        with estado_lock:
          q_actual = np.array([estado[joint] for joint in jnames])
          tiempo_estado = ultimo_estado[0]

        if time.monotonic() - tiempo_estado > 5.0:
          print("ERROR: No se reciben estados actualizados de Gazebo.")
          continue

        if not np.all(np.isfinite(q_actual)):
          print("ERROR: Estado articular inválido.")
          continue

        # Calcular cinemática directa deseada
        T = fkine_gripper(q_nuevo)

        print("\nq deseada [rad] =", np.round(q_nuevo, 3))
        print("Matriz T TCP deseada:")
        print(np.round(T, 3))

        # Verificar si ya está en la posición solicitada
        error_inicial = np.abs(q_nuevo - q_actual)

        if np.all(error_inicial <= TOLERANCIA_ARTICULAR):
          print("El robot ya se encuentra en la configuración solicitada.\n")
          continue

        # Obtener velocidades máximas de las articulaciones
        velocidades = np.array([
          float(simulation['joints'][joint]['velocity'])
          for joint in jnames
        ])

        if not np.all(np.isfinite(velocidades)) or np.any(velocidades <= 0):
          print("ERROR: Límites de velocidad inválidos.")
          continue

        # Calcular automáticamente el tiempo de movimiento
        desplazamiento = np.abs(q_nuevo - q_actual)

        # Interpolación cúbica con velocidades inicial y final nulas
        tiempo_estimado = 1.5 * np.max(desplazamiento / velocidades)

        # Margen de seguridad y duración mínima
        duracion = max(
          TIEMPO_MINIMO,
          0.1 + tiempo_estimado * FACTOR_SEGURIDAD
        )

        # Crear trayectoria articular
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = jnames

        # Punto inicial
        punto_inicial = JointTrajectoryPoint()
        punto_inicial.positions = q_actual.tolist()
        punto_inicial.velocities = [0.0] * 6
        punto_inicial.time_from_start = duracion_ros(0.1)

        # Punto final
        punto_final = JointTrajectoryPoint()
        punto_final.positions = q_nuevo.tolist()
        punto_final.velocities = [0.0] * 6
        punto_final.time_from_start = duracion_ros(duracion)

        goal.trajectory.points = [punto_inicial, punto_final]
        goal.goal_time_tolerance = duracion_ros(3.0)

        # Enviar trayectoria a Gazebo
        print("\nEnviando movimiento al controlador...")

        futuro = client.send_goal_async(goal)
        handle = esperar_futuro(futuro, TIMEOUT_CONEXION)

        if not handle.accepted:
          print("ERROR: El controlador rechazó la trayectoria.")
          continue

        print("Trayectoria aceptada. Robot en movimiento...")

        # Esperar que termine el movimiento
        resultado = handle.get_result_async()

        try:
          respuesta = esperar_futuro(
            resultado,
            duracion + TIMEOUT_RESULTADO
          )

        except TimeoutError:
          print("ERROR: Movimiento no completado dentro del tiempo esperado.")
          print("Solicitando cancelación de trayectoria...")

          cancelacion = handle.cancel_goal_async()

          try:
            esperar_futuro(cancelacion, 3.0)
          except (TimeoutError, RuntimeError):
            print("No se pudo confirmar la cancelación.")

          continue

        # Comprobar respuesta del controlador
        if (respuesta.status == GoalStatus.STATUS_SUCCEEDED
            and respuesta.result.error_code ==
            FollowJointTrajectory.Result.SUCCESSFUL):

          # Esperar una actualización del estado final
          time.sleep(0.2)

          with estado_lock:
            q_final = np.array([estado[joint] for joint in jnames])
            tiempo_final = ultimo_estado[0]

          if time.monotonic() - tiempo_final > 5.0:
            print("ERROR: No se pudo verificar el estado final de Gazebo.")
            continue

          if not np.all(np.isfinite(q_final)):
            print("ERROR: El estado final contiene valores inválidos.")
            continue

          # Comprobar error articular final
          error_articular = np.abs(q_nuevo - q_final)

          # Cinemática directa de la posición alcanzada
          T_final = fkine_gripper(q_final)

          print("\nConfiguración deseada [deg]:")
          print(np.round(np.rad2deg(q_nuevo), 2))

          print("Configuración alcanzada [deg]:")
          print(np.round(np.rad2deg(q_final), 2))

          print("Error articular [deg]:")
          print(np.round(np.rad2deg(error_articular), 4))

          print("\nMatriz T TCP alcanzada:")
          print(np.round(T_final, 3))

          if np.all(error_articular <= TOLERANCIA_ARTICULAR):
            print("\nMovimiento completado correctamente.\n")
          else:
            print("\nADVERTENCIA: El robot terminó fuera de la tolerancia articular.\n")

        else:
          print("\nERROR: La trayectoria no se completó.")
          print("Código:", respuesta.result.error_code)
          print("Detalle:", respuesta.result.error_string)

      except ValueError as e:
        print("Entrada inválida:", e)

      except TimeoutError as e:
        print("ERROR:", e)

      except KeyboardInterrupt:
        break

      except Exception as e:
        print("ERROR:", e)

  except (RuntimeError, TimeoutError, KeyboardInterrupt) as e:
    print("\nEjecución interrumpida:", e)

  finally:
    client.destroy()
    node.destroy_subscription(sub)
    node.destroy_node()

    if rclpy.ok():
      rclpy.shutdown()

    thread.join(timeout=2.0)

if __name__ == '__main__':
  main()