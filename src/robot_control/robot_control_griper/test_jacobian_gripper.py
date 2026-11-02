"""Compara Jacobianos de tool0 y TCP, sin tocar las FK originales."""
import numpy as np
from fkine_functions import fkine_hartech
from robot_control.jacobian_6dof import jacobian_6dof, singular_values_scaled
from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.jacobian_gripper import jacobian_gripper


def main():
    np.set_printoptions(precision=6, suppress=True)
    pruebas = {'HOME': [0, -50, 80, 0, -30, 0],
               'OTRA': [30, -45, 60, 0, -20, 0]}
    for nombre, qdeg in pruebas.items():
        q = np.deg2rad(qdeg)
        J0 = jacobian_6dof(q)
        Jt = jacobian_gripper(q)
        T0, Tt = fkine_hartech(q), fkine_gripper(q)
        s = singular_values_scaled(Jt)
        print(f'\n=== CON GRIPPER: {nombre} (TCP) ===')
        print('q [deg]:', qdeg)
        print('Posición tool0 [m]:', T0[:3, 3])
        print('Posición TCP [m]:', Tt[:3, 3])
        print('J completo 6x6 del TCP:\n', Jt)
        print('Diferencia angular (debe ser ~0):', np.linalg.norm(J0[3:] - Jt[3:]))
        print('Valores singulares escalados (L=0.3 m):', s)
        print('Razón sigma_min/sigma_max:', s[-1] / s[0])
        print('Rango numérico del Jacobiano sin escalar:', np.linalg.matrix_rank(Jt))
        # Verificación adicional: derivada lineal directa del TCP.
        h = 1e-5
        Jv_directo = np.zeros((3, 6))
        for i in range(6):
            delta = np.zeros(6); delta[i] = h
            Jv_directo[:, i] = (fkine_gripper(q + delta)[:3, 3] -
                               fkine_gripper(q - delta)[:3, 3]) / (2*h)
        error = np.linalg.norm(Jt[:3, :] - Jv_directo)
        print('Error de verificación de Jv TCP:', error)
        if not np.allclose(Jt[:3, :], Jv_directo, atol=1e-8):
            raise AssertionError('El Jacobiano lineal del TCP no coincide.')


if __name__ == '__main__':
    main()
