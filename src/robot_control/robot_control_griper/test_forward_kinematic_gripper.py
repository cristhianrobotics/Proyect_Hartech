#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
from markers import *
from fkine_functions import TF2xyzquat
from robot_control_griper.fkine_gripper import fkine_gripper
from sensor_msgs.msg import JointState

def main():

  rclpy.init()
  node = rclpy.create_node('ForwardKinematicsGripper')
  pub = node.create_publisher(JointState, 'joint_states', 10)
  
  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()
  
  #bmarker = BallMarker(node, color['GREEN'])
  marker = FrameMarker(node)
 
  # Joint names - HARTECH
  jnames = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'hartech_left_finger_joint']
  
  # Joint Configuration inicial - HOME HARTECH
  q = np.deg2rad([0, -50, 80, 0, -30, 0])

  # El marco calculado es hartech_tcp (100 mm desde tool0).
  # Lock para acceso seguro a q
  q_lock = threading.Lock()

  def actualizar_pose():
    T = fkine_gripper(q)
    print("\nMatriz T del TCP:")
    print(np.round(T, 3))
    x0 = TF2xyzquat(T)
    marker.setPose(x0)

  # Calcular pose inicial
  actualizar_pose()

  def leer_articulaciones():
    nonlocal q
    print("\nIngresa 6 articulaciones en GRADOS separadas por espacio.")
    print("Ejemplo: 0 -45 30 0 90 0")
    print("Escribe 'salir' para terminar.\n")

    while rclpy.ok():
      try:
        entrada = input("q[deg] > ").strip()

        if entrada.lower() in ['salir', 'exit', 'quit']:
          rclpy.shutdown()
          break

        valores = [float(v) for v in entrada.replace(',', ' ').split()]

        if len(valores) != 6:
          print("Debes ingresar exactamente 6 valores.")
          continue

        q_nuevo = np.deg2rad(np.array(valores))

        with q_lock:
          q = q_nuevo
          T = fkine_gripper(q)
          print("\nq [rad] =", np.round(q, 3))
          print("Matriz T del TCP:")
          print(np.round(T, 3))
          x0 = TF2xyzquat(T)
          marker.setPose(x0)

      except Exception as e:
        print("Entrada inválida:", e)

  # Hilo para leer articulaciones desde teclado
  input_thread = threading.Thread(target=leer_articulaciones, daemon=True)
  input_thread.start()
 
  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  
  # Loop rate (in Hz)
  rate = node.create_rate(20)

  # Continuous execution loop
  while rclpy.ok():
    with q_lock:
      q_actual = q.copy()

    # Current time (needed for ROS)
    jstate.header.stamp = node.get_clock().now().to_msg()
    jstate.position = q_actual.tolist() + [0.015]  # pinza fija en 30 mm para RViz

    # Publish the message
    pub.publish(jstate)
    #bmarker.publish()
    marker.publish()

    # Wait for the next iteration
    rate.sleep()
    
  
  node.destroy_node()
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()