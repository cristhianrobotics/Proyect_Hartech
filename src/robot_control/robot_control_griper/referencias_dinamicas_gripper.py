#!/usr/bin/env python3
"""Referencias articulares/Cartesianas para ensayos dinámicos de HARTECH+gripper.

No interactúa con ROS/Gazebo; reutiliza las funciones ya validadas del proyecto.
La trayectoria de cuadrado SE CALCULA en el TCP con orientación constante.
"""
import numpy as np
from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.jacobian_gripper import jacobian_gripper
from robot_control.ikine_pose_6d import errores_pose

DT = 0.02
Q_HOME = np.deg2rad([0., -50., 80., 0., -30., 0.])
Q_META = np.deg2rad([20., -55., 75., 10., -25., 10.])
Q_ESCRITURA = np.deg2rad([0., -55., 95., 0., 50., 0.])
Q_MIN = np.deg2rad([-160., -95., -110., -150., -70., -160.])
Q_MAX = np.deg2rad([160., 5., 110., 150., 70., 160.])
MAX_DQ = 0.75  # rad/s: simulación provisional, no datos físicos certificados.
MAX_DDQ = 3.0  # rad/s²: límite precautorio de la REFERENCIA.


def s_quintico(t, total):
    u = np.clip(t/total, 0., 1.)
    s = 10*u**3 - 15*u**4 + 6*u**5
    ds = (30*u**2-60*u**3+30*u**4)/total
    dds = (60*u-180*u**2+120*u**3)/(total**2)
    return s, ds, dds


def preparar_articular(q0, qdest, duracion, dt=DT):
    t = np.linspace(0., duracion, int(round(duracion/dt))+1)
    s, ds, dds = s_quintico(t, duracion)
    diff = qdest-q0
    return (t, q0[None,:]+s[:,None]*diff[None,:],
            ds[:,None]*diff[None,:], dds[:,None]*diff[None,:])


def pose_cuadrado(t, T0, lado=.06, tiempo_lado=5.):
    """Cuadrado horizontal de 6 cm, +X,+Y,-X,-Y; R_TCP vertical fija."""
    p0 = T0[:3, 3]
    pts = [p0, p0+[lado,0,0], p0+[lado,lado,0], p0+[0,lado,0], p0]
    t = float(np.clip(t, 0., 4*tiempo_lado))
    tramo = min(int(t/tiempo_lado), 3)
    s = s_quintico(t-tramo*tiempo_lado, tiempo_lado)[0]
    Td = np.eye(4)
    Td[:3,:3] = T0[:3,:3]
    Td[:3,3] = pts[tramo] + s*(pts[tramo+1]-pts[tramo])
    return Td


def _dls(J, e, damping=.008):
    Js = np.asarray(J,dtype=float).copy()
    Js[3:,:] *= .25
    es = np.asarray(e,dtype=float).copy()
    es[3:] *= .25
    return Js.T @ np.linalg.solve(Js@Js.T+damping**2*np.eye(6), es)


def plan_cuadrado(dt=DT, lado=.06, tiempo_lado=5., max_error_m=.0025):
    """Genera poses cartesianas y las convierte en q(t),dq(t),ddq(t) por IK incremental.

    Se valida CADA punto antes de permitir que un nodo lo envíe a Gazebo.
    No hace planificación de colisiones (modelo en mesa virtual sin contacto).
    """
    if not (0 < lado <= .08 and tiempo_lado >= 3.0):
        raise ValueError('Se requiere 0 < lado <= 0.08 m y tiempo_lado >= 3 s.')
    T0 = fkine_gripper(Q_ESCRITURA)
    if np.dot(T0[:3,2], np.array([0.,0.,-1.])) < .99:
        raise ValueError('La postura de escritura NO orienta el TCP hacia abajo.')
    if T0[2,3] < .06:
        raise ValueError('El TCP nominal esta demasiado bajo para la prueba.')
    t = np.linspace(0.,4*tiempo_lado,int(round(4*tiempo_lado/dt))+1)
    q = Q_ESCRITURA.copy()
    qs=[]; errp=[]; errr=[]
    for ti in t:
        Td = pose_cuadrado(ti,T0,lado,tiempo_lado)
        # Newton local: cambia solo lo imprescindible de la configuracion anterior.
        for it in range(14):
            Tc = fkine_gripper(q)
            ep,er=errores_pose(Td,Tc)
            if np.linalg.norm(ep)<4e-5 and np.linalg.norm(er)<2e-4:
                break
            dq = _dls(jacobian_gripper(q),np.r_[ep,er])
            if np.linalg.norm(dq)>.12:
                dq *= .12/np.linalg.norm(dq)
            q = np.clip(q+dq,Q_MIN,Q_MAX)
        ep,er=errores_pose(Td,fkine_gripper(q))
        errp.append(np.linalg.norm(ep));errr.append(np.linalg.norm(er))
        if errp[-1]>max_error_m or errr[-1]>np.deg2rad(.5):
            raise ValueError('IK del cuadrado falla t=%.2fs: XYZ %.2fmm, ang %.2fdeg' %
                             (ti,1000*errp[-1],np.rad2deg(errr[-1])))
        if np.any(q < Q_MIN+1e-8) or np.any(q > Q_MAX-1e-8):
            raise ValueError('La referencia toca limites articulares durante el cuadrado.')
        qs.append(q.copy())
    qs=np.array(qs)
    dqs = np.gradient(qs,dt,axis=0,edge_order=2)
    ddqs = np.gradient(dqs,dt,axis=0,edge_order=2)
    # Anular los residuos numericos de velocidad/acc de parada.
    for idx in (0, int(round(tiempo_lado/dt)), int(round(2*tiempo_lado/dt)),
                int(round(3*tiempo_lado/dt)), len(t)-1):
        dqs[idx,:]=0.
    if np.max(np.abs(dqs)) > MAX_DQ or np.max(np.abs(ddqs)) > MAX_DDQ:
        raise ValueError('Trayectoria exige velocidades/aceleraciones demasiado grandes; '
                         'aumentar el tiempo por lado.')
    return t, qs, dqs, ddqs, np.max(errp),np.max(errr)


def plan_completo(tipo='regulacion',dt=DT,tiempo_lado=5.):
    """Devuelve (t,q_d,dq_d,ddq_d); siempre comienza desde HOME.

    regulacion: q_HOME -> q_META con quintico en 7s;
    trayectoria: HOME -> META -> HOME con quinticos;
    cuadrado: HOME -> POSTURA_ESCRITURA -> 6x6cm XY manteniendo +Z TCP abajo.
    """
    if tipo=='regulacion':
        resultado = preparar_articular(Q_HOME,Q_META,7.,dt)
    elif tipo=='trayectoria':
        a = preparar_articular(Q_HOME,Q_META,8.,dt)
        b = preparar_articular(Q_META,Q_HOME,8.,dt)
        resultado=(np.r_[a[0],8.+b[0][1:]],
                   np.vstack((a[1],b[1][1:])),np.vstack((a[2],b[2][1:])),
                   np.vstack((a[3],b[3][1:])))
    elif tipo=='cuadrado':
        a=preparar_articular(Q_HOME,Q_ESCRITURA,8.,dt)
        tc,qc,dqc,ddqc,ep,er=plan_cuadrado(dt,tiempo_lado=tiempo_lado)
        resultado=(np.r_[a[0],8.+tc[1:]],
                   np.vstack((a[1],qc[1:])),np.vstack((a[2],dqc[1:])),
                   np.vstack((a[3],ddqc[1:])))
    else:
        raise ValueError('Tipo de trayectoria desconocido: '+tipo)
    t,q,dq,ddq=resultado
    if np.any(q<Q_MIN-1e-6) or np.any(q>Q_MAX+1e-6):
        raise ValueError('Referencia fuera de limites articulares.')
    if np.max(np.abs(dq))>MAX_DQ+1e-6 or np.max(np.abs(ddq))>MAX_DDQ+1e-6:
        raise ValueError('Referencia articular excede velocidad/aceleracion nominal.')
    return t,q,dq,ddq


def evaluar_referencia(t,plan):
    """Interpolacion lineal temporal de q,dq,ddq ya muestreadas a dt fijo."""
    ts,qs,dqs,ddqs=plan
    t=float(np.clip(t,ts[0],ts[-1]))
    idx=min(int(np.searchsorted(ts,t,side='right')-1),len(ts)-2)
    a=(t-ts[idx])/(ts[idx+1]-ts[idx])
    return tuple((1-a)*arr[idx]+a*arr[idx+1] for arr in (qs,dqs,ddqs))
