#!/usr/bin/env python3
"""Gráficas de XYZ y error temporal para los tres casos PD+g del Lab 7."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = Path.home() / 'Hartech_ws' / 'resultados_lab7_gripper'
MODOS = {'sub': 'Amortiguamiento bajo (nominal)',
         'critico': 'Amortiguamiento medio (nominal)',
         'sobre': 'Amortiguamiento alto (nominal)'}


def main():
    curvas = {}
    for modo, titulo in MODOS.items():
        ruta = BASE / f'pd_gripper_{modo}.csv'
        if not ruta.is_file():
            print(f'Falta {ruta.name}. Ejecuta primero: --modo todos')
            continue
        datos = np.genfromtxt(ruta, delimiter=',', names=True)
        datos = np.atleast_1d(datos)
        curvas[modo] = datos
        fig, ax = plt.subplots(figsize=(10, 5))
        for i, eje in enumerate('xyz'):
            ax.plot(datos['t'], datos[eje], label=f'{eje.upper()} real')
            ax.plot(datos['t'], datos[eje+'d'], '--', label=f'{eje.upper()} deseada')
        ax.set(xlabel='Tiempo [s]', ylabel='Posición TCP [m]',
               title=f'HARTECH con gripper — {titulo}')
        ax.grid(True); ax.legend(ncol=3)
        fig.tight_layout()
        salida = BASE / f'xyz_pd_gripper_{modo}.png'
        fig.savefig(salida, dpi=180)
        plt.close(fig)
        print('Gráfica:', salida)
    if curvas:
        fig, ax = plt.subplots(figsize=(10, 5))
        for modo, datos in curvas.items():
            ax.plot(datos['t'], 1000 * datos['error_xyz_m'], label=MODOS[modo])
        ax.set(xlabel='Tiempo [s]', ylabel='Error cartesiano del TCP [mm]',
               title='Comparación PD + gravedad — HARTECH con gripper')
        ax.grid(True); ax.legend()
        fig.tight_layout()
        salida = BASE / 'comparacion_error_pd_gripper.png'
        fig.savefig(salida, dpi=180)
        plt.close(fig)
        print('Comparación:', salida)


if __name__ == '__main__':
    main()
