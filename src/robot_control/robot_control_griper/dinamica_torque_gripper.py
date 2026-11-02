#!/usr/bin/env python3
"""Leyes dinámicas del Lab 7, reutilizando Pinocchio de HARTECH + gripper.

pdg: PD + g, reproduccion del laboratorio, SIN integral.
pidg: PD + g + compensacion integral opcional para error estacionario Gazebo.
ctc: computed torque, compensa M,C,g mediante RNEA; integral opcional.

El término integral NO pertenece al PD original; se reporta explicitamente.
Esta biblioteca NO aplica torques ni necesita ROS.
"""
import numpy as np
import pinocchio as pin
from robot_control_griper.modelo_dinamico_gripper import cargar_modelo_con_gripper
from robot_control_griper.referencias_dinamicas_gripper import Q_HOME

ESFUERZO_MAX = np.array([50.,50.,30.,10.,10.,5.])
TORQUE_PERMITIDO = .75*ESFUERZO_MAX  # conservador, provisional para simulacion
DT=.02


def crear_control(model=None,modo='ctc',omega=2.2,zeta=1.15,integral=False):
    if model is None:
        model=cargar_modelo_con_gripper(30.)
    return ControlDinamico(model,modo,omega,zeta,integral)


class ControlDinamico:
    def __init__(self,model,modo='ctc',omega=2.2,zeta=1.15,integral=False):
        if modo not in ('pdg','ctc'):
            raise ValueError('modo debe ser pdg o ctc')
        self.model=model
        self.data=model.createData()
        self.modo=modo
        self.omega=float(omega)
        self.zeta=float(zeta)
        self.integral_habilitada=bool(integral)
        self.integral=np.zeros(6)
        self.v_filtrada=None
        self.saturaciones=0
        self.ultimo_torque=None
        M=pin.crba(model,self.data,Q_HOME)
        diag=np.diag(np.triu(M)+np.triu(M,1).T)
        if (model.nq!=6 or model.nv!=6 or np.any(diag<=0) or
            not np.all(np.isfinite(diag))):
            raise ValueError('Modelo dinámico invalido, se esperaba seis ejes y M positiva.')
        if modo=='pdg':
            self.kp=np.maximum(2.*diag*2.6**2,[1.,1.5,1.,.20,.30,.12])
            self.kd=np.maximum(2.5*2.*diag*2.6,[.30,.80,.55,.10,.12,.045])
            # KI en Nm/(rad*s); correccion max integrada MUY limitada.
            self.ki=np.array([.12,.18,.12,.025,.025,.020])
            self.limite_i=np.array([.6,1.0,.6,.20,.20,.08])
        else:
            # wn rad/s y damping adimensional, expresadas en unidades de aceleración.
            self.kp=np.ones(6)*self.omega**2
            self.kd=np.ones(6)*(2*self.zeta*self.omega)
            self.ki=np.ones(6)*.30  # 1/s³: opcional, NO dinámica inversa clásica.
            self.limite_i=np.ones(6)*.60  # max aporte en aceleración [rad/s²]

    def calcular(self,q,dq,qr,dqr,ddqr,dt=DT):
        q,dq,qr,dqr,ddqr=(np.asarray(v,dtype=float).reshape(6) for v in
                           (q,dq,qr,dqr,ddqr))
        if not all(np.all(np.isfinite(v)) for v in (q,dq,qr,dqr,ddqr)):
            raise ValueError('Estado o referencia contiene NaN o infinito.')
        if self.v_filtrada is None:
            self.v_filtrada=dq.copy()
        else:
            alpha=.20
            self.v_filtrada=alpha*dq+(1-alpha)*self.v_filtrada
        e=qr-q
        ed=dqr-self.v_filtrada
        # Banda anti-acumulacion cerca de limites y correccion integral acotada.
        if self.integral_habilitada:
            self.integral += np.clip(e,-.5,.5)*max(0.,min(float(dt),.04))
            self.integral=np.clip(self.integral,-self.limite_i/self.ki,
                                  self.limite_i/self.ki)
            i_term=self.ki*self.integral
        else:
            i_term=np.zeros(6)
        g=pin.computeGeneralizedGravity(self.model,self.data,q).copy()
        if self.modo=='pdg':
            tau0=self.kp*e+self.kd*ed+i_term+g
        else:
            a_cmd=ddqr+self.kp*e+self.kd*ed+i_term
            tau0=pin.rnea(self.model,self.data,q,dq,a_cmd).copy()
        if not np.all(np.isfinite(tau0)):
            raise ValueError('Torque calculado no finito.')
        sat=np.abs(tau0)>TORQUE_PERMITIDO
        tau=np.clip(tau0,-TORQUE_PERMITIDO,TORQUE_PERMITIDO)
        if np.any(sat):
            self.saturaciones+=1
            # Anti-windup: deshacer integracion en los ejes saturados, si empeora saturación.
            if self.integral_habilitada:
                self.integral[sat & (np.sign(e)==np.sign(tau0))] -= np.clip(
                    e[sat & (np.sign(e)==np.sign(tau0))],-.5,.5)*max(0.,min(float(dt),.04))
        return tau, {'e':e,'ed':ed,'g':g,'raw':tau0,'sat':bool(np.any(sat)),
                     'integral_nm':i_term.copy(), 'dq_f':self.v_filtrada.copy()}

    def reiniciar(self):
        self.integral[:]=0.
        self.v_filtrada=None
        self.saturaciones=0
        self.ultimo_torque=None
