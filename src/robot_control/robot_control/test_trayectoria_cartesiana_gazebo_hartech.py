#!/usr/bin/env python3
"""HARTECH SIN GRIPPER: seguimiento de un cuadrado cartesiano en Gazebo.

Cuadrado de 4 x 4 cm en el plano XZ de base_link, con Y y orientacion de
`tool0` constantes. Se parte de la pose REAL al iniciar, cerca de HOME.
Usa la ley de control diferencial 6D y arm_controller del nodo anterior.
No modifica otros archivos ni implementa dibujo fisico con una herramienta.
"""
import numpy as np
from fkine_functions import fkine_hartech
from robot_control.test_diffkine_pose_gazebo_hartech import controlar_gazebo

LADO = 0.04                 # metros: 4 cm
TIEMPO_POR_LADO = 5.0       # segundos nominales por lado
TIEMPO_TOTAL = 4 * TIEMPO_POR_LADO
Q_HOME = np.deg2rad([0, -50, 80, 0, -30, 0])


def trayectoria_cuadrado(t, Tinicio, Tfinal):
    """Pose, velocidad lineal y velocidad angular de la referencia en base_link.

    P0 -> P1 -> P2 -> P3 -> P0, manteniendo Y y R de Tinicio.
    Tfinal se ignora: esta tarea consiste en una trayectoria cerrada.
    """
    if not np.all(np.isfinite(Tinicio)):
        raise ValueError('Pose inicial no finita.')
    if abs(float(t)) < 1e-12:
        # El cuadrado solo ha sido verificado partiendo cerca de HOME.
        Thome = fkine_hartech(Q_HOME)
        dp = np.linalg.norm(Tinicio[:3, 3] - Thome[:3, 3])
        Rrel = Tinicio[:3, :3] @ Thome[:3, :3].T
        ang = np.arccos(np.clip((np.trace(Rrel) - 1.0)/2.0, -1.0, 1.0))
        if dp > 0.035 or ang > np.deg2rad(12):
            raise ValueError('Para esta prueba, reinicia Gazebo con pose:=home.')

    p0 = Tinicio[:3, 3]
    p1 = p0 + np.array([-LADO, 0.0, 0.0])
    p2 = p0 + np.array([-LADO, 0.0, LADO])
    p3 = p0 + np.array([0.0, 0.0, LADO])
    vertices = (p0, p1, p2, p3, p0)

    t = float(np.clip(t, 0.0, TIEMPO_TOTAL))
    lado = min(int(t / TIEMPO_POR_LADO), 3)
    u = (t - lado * TIEMPO_POR_LADO) / TIEMPO_POR_LADO
    s = 3*u*u - 2*u*u*u  # inicio y fin de cada lado con velocidad cero
    ds = 6*u*(1-u) / TIEMPO_POR_LADO
    delta = vertices[lado + 1] - vertices[lado]

    Td = np.eye(4)
    Td[:3, :3] = Tinicio[:3, :3]   # orientacion CONSTANTE
    Td[:3, 3] = vertices[lado] + s*delta
    v_ff = ds * delta
    w_ff = np.zeros(3)
    return Td, v_ff, w_ff


def main():
    print('\n=== HARTECH | CUADRADO CARTESIANO EN GAZEBO ===')
    print('Plano: XZ | Lado: 4 cm | Y y orientacion constantes')
    print('Recorrido: P0 -> P1 -> P2 -> P3 -> P0')
    print('Duracion nominal: 20 s; requiere empezar cerca de HOME.')
    controlar_gazebo(
        trayectoria_cuadrado,
        duracion=TIEMPO_TOTAL,
        etiqueta='trayectoria_cuadrado_hartech_gazebo',
        registro='trayectoria_cuadrado_gazebo.csv',
    )


if __name__ == '__main__':
    main()

