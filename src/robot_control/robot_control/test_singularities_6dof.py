#!/usr/bin/env python3
"""Análisis del Jacobiano 6x6 de HARTECH sin mover el robot."""
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Genera PNG sin abrir ventanas.
import matplotlib.pyplot as plt

from robot_control.jacobian_6dof import jacobian_6dof, singular_values_scaled

# Configuración HOME válida; se modifica exclusivamente J5.
HOME = np.array([0., -50., 80., 0., -30., 0.])
LONGITUD = 0.3  # [m], para comparar componentes lineales y angulares.
TOL_RANGO = 1e-6


def analizar_singularidades(nombre, funcion_jacobiano, archivo_png):
    print(f'\n=== SINGULARIDADES HARTECH: {nombre} ===')
    print('Configuración base [deg]:', HOME)
    print('Se modifica J5 entre -70° y 70°. Los demás ejes permanecen fijos.')
    print('Análisis numérico únicamente: NO se enviarán comandos al robot.\n')
    print(f'{"J5[°]":>8} {"sigma_min":>13} {"sigma_min/max":>15} {"rango":>7}  observación')

    valores = np.linspace(-70., 70., 141)
    minimos = []
    razones = []
    rangos = []
    mostrar = {-70, -30, -5, -1, 0, 1, 5, 30, 70}
    for j5 in valores:
        qdeg = HOME.copy()
        qdeg[4] = j5
        J = funcion_jacobiano(np.deg2rad(qdeg))
        sigmas = singular_values_scaled(J, length=LONGITUD)
        sigma_min = float(sigmas[-1])
        razon = sigma_min / float(sigmas[0])
        rango = int(np.linalg.matrix_rank(J, tol=TOL_RANGO))
        minimos.append(sigma_min)
        razones.append(razon)
        rangos.append(rango)
        if int(round(j5)) in mostrar:
            observacion = 'SINGULAR (rango < 6)' if rango < 6 else 'Rango completo'
            print(f'{j5:8.0f} {sigma_min:13.8f} {razon:15.8f} {rango:7d}  {observacion}')

    i0 = int(np.argmin(np.abs(valores)))
    q0 = HOME.copy()
    q0[4] = 0.
    J0 = funcion_jacobiano(np.deg2rad(q0))
    eje4 = J0[3:, 3] / np.linalg.norm(J0[3:, 3])
    eje6 = J0[3:, 5] / np.linalg.norm(J0[3:, 5])
    alineamiento = abs(float(eje4 @ eje6))
    print('\nEn J5=0°:')
    print('Rango de J:', rangos[i0])
    print('Sigma mínima escalada:', f'{minimos[i0]:.3e}')
    print('Alineación absoluta entre ejes J4 y J6:', f'{alineamiento:.8f}')
    print('Interpretación: J4 y J6 están alineados; la muñeca pierde una dirección instantánea de giro.')
    print('IMPORTANTE: no enviar movimientos al robot usando esta prueba.')

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(valores, razones, linewidth=2, label='σmin/σmax (Jacobiano escalado)')
    ax.axvline(0, linestyle='--', label='J5 = 0° (singularidad)')
    ax.set(xlabel='Ángulo de J5 [grados]', ylabel='Relación σmin/σmax',
           title=f'Singularidad de muñeca — HARTECH {nombre}')
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(archivo_png, dpi=180)
    plt.close(fig)
    print('\nGráfica guardada:', archivo_png)
    print('Nota: la relación depende de la longitud de escalado (0.3 m).')
    return np.asarray(valores), np.asarray(razones), np.asarray(rangos)


def main():
    analizar_singularidades('SIN GRIPPER (tool0)', jacobian_6dof,
                           'singularidades_hartech_sin_gripper.png')


if __name__ == '__main__':
    main()
