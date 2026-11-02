#!/usr/bin/env python3
"""Mismas gráficas del Lab 6 UR5, con rutas adaptadas para HARTECH."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # necesario para Matplotlib 3D


def cargar_datos(nombre_archivo):
    data = np.loadtxt(nombre_archivo)
    if data.ndim == 1:
        if data.size % 3 != 0:
            raise ValueError(f'El archivo {nombre_archivo} no tiene datos múltiplos de 3.')
        data = data.reshape(-1, 3)
    if data.shape[0] == 3 and data.shape[1] != 3:
        data = data.T
    if data.shape[1] != 3:
        raise ValueError(f'El archivo {nombre_archivo} debe tener 3 columnas (x,y,z).')
    return data


def main():
    ruta = Path.home() / 'Hartech_ws' / 'resultados_lab6'
    x_current = cargar_datos(ruta / 'xcurrent.txt')
    x_desired = cargar_datos(ruta / 'xdesired.txt')
    n = min(len(x_current), len(x_desired))
    x_current, x_desired = x_current[:n], x_desired[:n]
    t = np.arange(n)

    fig1 = plt.figure(figsize=(8, 6))
    ax = fig1.add_subplot(111, projection='3d')
    ax.plot(x_current[:, 0], x_current[:, 1], x_current[:, 2], label='Posición actual')
    ax.plot(x_desired[:, 0], x_desired[:, 1], x_desired[:, 2], '--', label='Posición deseada')
    ax.scatter(*x_current[0], marker='o', label='Inicio actual')
    ax.scatter(*x_current[-1], marker='^', label='Fin actual')
    ax.scatter(*x_desired[-1], marker='s', label='Posición deseada')
    ax.set_title('Espacio cartesiano 3D - HARTECH')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.legend()
    ax.grid(True)

    fig2 = plt.figure(figsize=(10, 6))
    ax2 = fig2.add_subplot(111)
    for i, eje in enumerate(('x', 'y', 'z')):
        ax2.plot(t, x_current[:, i], label=f'{eje} actual')
        ax2.plot(t, x_desired[:, i], '--', label=f'{eje} deseada')
    ax2.set_title('Cambio de posición cartesiana - HARTECH')
    ax2.set_xlabel('Muestras')
    ax2.set_ylabel('Posición [m]')
    ax2.legend()
    ax2.grid(True)

    fig1.savefig(ruta / 'grafica_3D.png', dpi=300, bbox_inches='tight')
    fig2.savefig(ruta / 'grafica_2D_xyz.png', dpi=300, bbox_inches='tight')
    print('Gráficas guardadas en:', ruta)
    plt.show()


if __name__ == '__main__':
    main()
