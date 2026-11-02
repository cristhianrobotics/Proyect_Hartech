"""Prueba en terminal del Jacobiano completo del brazo HARTECH (tool0)."""
import numpy as np
from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof, singular_values_scaled


def main():
    np.set_printoptions(precision=6, suppress=True)
    pruebas = {'HOME': [0, -50, 80, 0, -30, 0],
               'OTRA': [30, -45, 60, 0, -20, 0]}
    for nombre, qdeg in pruebas.items():
        q = np.deg2rad(qdeg)
        T = fkine_hartech(q)
        J = jacobian_6dof(q)
        s = singular_values_scaled(J)
        print(f'\n=== SIN GRIPPER: {nombre} (tool0) ===')
        print('q [deg]:', qdeg)
        print('Posición tool0 [m]:', T[:3, 3])
        print('J completo 6x6 (filas 1-3 lineales, 4-6 angulares):\n', J)
        print('Valores singulares escalados (L=0.3 m):', s)
        print('Razón sigma_min/sigma_max:', s[-1] / s[0])
        print('Rango numérico del Jacobiano sin escalar:', np.linalg.matrix_rank(J))


if __name__ == '__main__':
    main()
