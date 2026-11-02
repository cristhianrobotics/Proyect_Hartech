#!/usr/bin/env python3
"""Graficas de CSV de control por esfuerzo HARTECH con gripper.

Para un archivo dibuja errores de seguimiento y trayectoria XY deseada/real.
Para varios archivos añade comparativa PD+I+g frente a computed torque.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
from robot_control_griper.fkine_gripper import fkine_gripper


def leer(path):
    with open(path,newline='',encoding='utf-8') as f:
        filas=list(csv.DictReader(f))
    if not filas:raise ValueError('CSV vacio: '+str(path))
    t=np.array([float(r['t_s']) for r in filas])
    e_q=np.array([float(r['error_q_deg']) for r in filas])
    e_tcp=np.array([float(r['error_tcp_mm']) for r in filas])
    q=np.array([[float(r['q%d_rad'%i]) for i in range(1,7)] for r in filas])
    qr=np.array([[float(r['qd%d_rad'%i]) for i in range(1,7)] for r in filas])
    tau=np.array([[float(r['tau%d_Nm'%i]) for i in range(1,7)] for r in filas])
    sat=np.array([int(r['saturado']) for r in filas])
    return t,e_q,e_tcp,q,qr,tau,sat


def main():
    p=argparse.ArgumentParser()
    p.add_argument('csv',nargs='*',help='CSV generados por test_control_dinamico_gripper_gazebo')
    args=p.parse_args()
    base=Path.home()/'Hartech_ws'/'resultados_lab7_gripper'
    archivos=[Path(s) for s in args.csv]
    if not archivos:
        archivos=sorted(base.glob('torque_*.csv'),key=lambda p:p.stat().st_mtime)[-4:]
    if not archivos:raise SystemExit('No hay CSV. Ejecuta primero el controlador Gazebo.')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,1,figsize=(10,8),sharex=True)
    for path in archivos:
        t,eq,et,*_=leer(path)
        axs[0].plot(t,eq,label=path.stem)
        axs[1].plot(t,et,label=path.stem)
        print(f'{path.name}: error final q={eq[-1]:.3f}deg TCP={et[-1]:.2f}mm')
    axs[0].set_ylabel('Error articular [grados]')
    axs[1].set_ylabel('Error TCP [mm]');axs[1].set_xlabel('Tiempo [s]')
    for ax in axs: ax.grid(True);ax.legend(fontsize=8)
    fig.suptitle('HARTECH gripper: comparacion de control dinamico por torque')
    fig.tight_layout()
    salida=base/'comparacion_gazebo_control_dinamico.png'
    fig.savefig(salida,dpi=160);plt.close(fig)
    print('Grafica:',salida)
    # Describir cuadrado SOLO si archivo más reciente contiene cuadrado.
    square=[x for x in archivos if 'cuadrado' in x.stem]
    if square:
        path=square[-1]
        t,eq,et,q,qr,tau,sat=leer(path)
        real=np.array([fkine_gripper(qq)[:3,3] for qq in q])
        des=np.array([fkine_gripper(qq)[:3,3] for qq in qr])
        fig,ax=plt.subplots(figsize=(7,7))
        ax.plot(des[:,0]*100,des[:,1]*100,'--',label='Cuadrado objetivo')
        ax.plot(real[:,0]*100,real[:,1]*100,label='TCP en Gazebo')
        ax.scatter([des[0,0]*100],[des[0,1]*100],label='Inicio (incluye preparacion)')
        ax.set_xlabel('X [cm]');ax.set_ylabel('Y [cm]')
        ax.set_title('HARTECH gripper: trayectoria XY (incluye preparacion)')
        ax.axis('equal');ax.grid(True);ax.legend()
        fig.tight_layout();salida=base/'cuadrado_dinamico_gripper_xy.png'
        fig.savefig(salida,dpi=160);plt.close(fig)
        print('Grafica:',salida)

if __name__=='__main__':main()
