#!/usr/bin/env python3
"""LAB7: comparar PD+g, PD+I+g y computed-torque con simulacion ABA ideal.
No usa Gazebo, ni modifica URDF/launch ni publica joint_states.
"""
import csv
from pathlib import Path
import numpy as np
import pinocchio as pin
from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.dinamica_torque_gripper import crear_control
from robot_control_griper.referencias_dinamicas_gripper import (
    Q_HOME,Q_MIN,Q_MAX,plan_completo,evaluar_referencia)

DT=.005
T_EXTRA=10.


def simular(modo,plan,model):
    ctrl=crear_control(model,modo='pdg' if modo in ('pdg','pidg') else 'ctc',
                       integral=(modo=='pidg'))
    data=model.createData()
    q=Q_HOME.copy();dq=np.zeros(6)
    tf=plan[0][-1]+T_EXTRA
    tc=np.arange(0,tf+DT/2,DT)
    rows=[]
    for i,t in enumerate(tc):
        qr,dqr,ddqr=evaluar_referencia(t,plan)
        tau,info=ctrl.calcular(q,dq,qr,dqr,ddqr,dt=DT)
        ddq=pin.aba(model,data,q,dq,tau).copy()
        if not np.all(np.isfinite(ddq)) or np.max(np.abs(ddq))>30:
            raise RuntimeError('Aceleracion excesiva en modo '+modo)
        # Integracion semi-implicita de Euler, util en sistemas mecanicos.
        dq=dq+DT*ddq
        q=q+DT*dq
        if np.any(q<Q_MIN-.05) or np.any(q>Q_MAX+.05) or np.max(np.abs(dq))>2:
            raise RuntimeError('La prueba supera un limite en modo '+modo)
        if i%20==0 or i==len(tc)-1:
            e_q=np.rad2deg(np.linalg.norm(qr-q))
            e_tcp=1000*np.linalg.norm(fkine_gripper(qr)[:3,3]-fkine_gripper(q)[:3,3])
            rows.append((t,e_q,e_tcp,max(abs(dq)),max(abs(tau)),int(info['sat'])))
    return rows


def main():
    base=Path.home()/'Hartech_ws'/'resultados_lab7_gripper'
    base.mkdir(parents=True,exist_ok=True)
    model=crear_control().model
    plan=plan_completo('regulacion',dt=.02)
    series={}
    for modo in ('pdg','pidg','ctc'):
        datos=simular(modo,plan,model)
        series[modo]=np.asarray(datos)
        ruta=base/('comparacion_aba_'+modo+'.csv')
        with ruta.open('w',newline='',encoding='utf-8') as f:
            w=csv.writer(f)
            w.writerow(['t_s','error_q_deg','error_tcp_mm','max_dq_rad_s','max_tau_nm','saturado'])
            w.writerows(datos)
        u=series[modo][-1]
        print('%s: error articular %.4f deg | TCP %.4f mm | saturaciones %d | %s'%
              (modo,u[1],u[2],int(sum(series[modo][:,5])),ruta))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        for col,lab,name in ((1,'Error articular [grados]','comparacion_aba_error_articular.png'),
                             (2,'Error TCP [mm]','comparacion_aba_error_tcp.png')):
            fig,ax=plt.subplots(figsize=(9,5))
            for modo,dat in series.items():ax.plot(dat[:,0],dat[:,col],label=modo)
            ax.set_xlabel('Tiempo [s]');ax.set_ylabel(lab)
            ax.set_title('HARTECH con gripper: PD+g vs PD+I+g vs dinámica inversa (ABA ideal)')
            ax.grid(True);ax.legend();fig.tight_layout()
            fig.savefig(base/name,dpi=150);plt.close(fig)
            print('Grafica:',base/name)
    except ImportError:
        print('Matplotlib no disponible; CSV guardados.')
    print('Nota: ABA usa el MISMO modelo que el controlador. Gazebo requiere validacion aparte.')


if __name__=='__main__':main()
