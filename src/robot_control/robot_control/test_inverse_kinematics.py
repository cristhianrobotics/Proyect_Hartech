#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from markers import *
from ikine_functions import *
from sensor_msgs.msg import JointState

# Grafica
def graficar_error(ee):
  # crear nombre único para no sobreescribir archivos
  nombre = "error_ikine_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"

  # crear figura
  plt.figure()
  plt.plot(np.abs(ee), 'b')
  plt.plot(np.abs(ee), 'b.')
  plt.title("Evolución del error")
  plt.xlabel("Número de iteraciones")
  plt.ylabel("Norma del error")
  plt.grid()

  # guardar gráfica
  plt.savefig(nombre, dpi=300, bbox_inches='tight')
  plt.close()

  print(f"Gráfica del error guardada en: {nombre}")


def main():
  rclpy.init()
  node = rclpy.create_node('InverseKinematics')
  pub = node.create_publisher(JointState, 'joint_states', 10)

  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()

  # Marcador tipo esfera para el efector final
  marker = BallMarker(node, color['GREEN'], scale=0.025)

  # Joint names - HARTECH
  jnames = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

  # Joint Configuration inicial - HOME HARTECH
  q = np.deg2rad([0, -50, 80, 0, -30, 0])

  # Lock para acceso seguro a q
  q_lock = threading.Lock()

  def actualizar_pose():
    T = fkine_hartech(q)
    print("\nMatriz T actual:")
    print(np.round(T, 3))
    marker.position(T)

  # mostrar pose inicial
  actualizar_pose()

  def leer_posicion_cartesiana():
    nonlocal q
    print("\nIngresa la posición deseada del efector final en metros.")
    print("Formato: x y z")
    print("Ejemplo: 0.0 0.0 0.721")
    print("Escribe 'salir' para terminar.\n")

    while rclpy.ok():
      try:
        entrada = input("xdes[m] > ").strip()

        if entrada.lower() in ['salir', 'exit', 'quit']:
          rclpy.shutdown()
          break

        valores = [float(v) for v in entrada.replace(',', ' ').split()]

        if len(valores) != 3:
          print("Debes ingresar exactamente 3 valores: x y z")
          continue

        xdes = np.array(valores)

        with q_lock:
          q_inicial = q.copy()

        # calcular cinemática inversa
        q_sol, ee = ikine(xdes, q_inicial)    # Metodo Newton

        # graficar y guardar el error
        graficar_error(ee)

        # verificar convergencia antes de publicar
        if not np.all(np.isfinite(q_sol)):
          print("No se obtuvo una solución numérica válida.")
          continue

        T = fkine_hartech(q_sol)
        error_final = np.linalg.norm(xdes - T[0:3, 3])

        if error_final >= 0.001:
          print("La cinemática inversa no convergió.")
          print("Error final:", np.round(error_final, 6))
          continue

        with q_lock:
          q = q_sol

        # calcular pose final alcanzada
        x_final = T[0:3, 3]

        print("\nPosición deseada:", np.round(xdes, 4))
        print("Posición alcanzada:", np.round(x_final, 4))
        print("Error inicial:", np.round(ee[0], 6))
        print("Error final:", np.round(error_final, 6))
        print("q solución [rad]:", np.round(q, 4))
        print("q solución [deg]:", np.round(np.rad2deg(q), 2))

        marker.position(T)

      except Exception as e:
        print("Entrada inválida:", e)

  # hilo para leer la posición desde teclado
  input_thread = threading.Thread(target=leer_posicion_cartesiana, daemon=True)
  input_thread.start()

  # mensaje JointState
  jstate = JointState()
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames

  # frecuencia del loop
  rate = node.create_rate(20)

  # loop principal
  while rclpy.ok():
    with q_lock:
      q_actual = q.copy()

    jstate.header.stamp = node.get_clock().now().to_msg()
    jstate.position = q_actual.tolist()

    pub.publish(jstate)
    marker.publish()
    rate.sleep()

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()