#!/usr/bin/env python3
"""Control dinamico de HARTECH+gripper via torque (Gazebo Classic 11).

Requiere Gazebo iniciado con:
  ros2 launch hartech_tools gazebo_esfuerzo.launch.py autostart_pd:=false
El mundo arranca a gravedad 0; ESTE nodo la restablece progresivamente
con control de esfuerzo ACTIVO durante toda la transicion.
Revision CSV J2/J3: conserva rampa V3 y modos regulacion/trayectoria.
SOLO ctc+cuadrado: verificacion HOME a 1g, realimentacion suplementaria
en Nm por eje, correccion integral acotada, avance supervisado y timeout.
Ajuste experimental basado en CSV V5; NO validado aun en Gazebo.
No modifica el generador del cuadrado ni los modos regulacion/trayectoria.
Controla brazo 6 DOF, NO la apertura de los dedos. No publicar /joint_states.
"""
import argparse
import csv
import shutil
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_control_griper.fkine_gripper import fkine_gripper
from robot_control_griper.dinamica_torque_gripper import crear_control, TORQUE_PERMITIDO
from robot_control_griper.referencias_dinamicas_gripper import (
    Q_HOME,Q_MIN,Q_MAX, DT, plan_completo,evaluar_referencia)

JOINTS=[f'joint_{i}' for i in range(1,7)]
MAX_AGE=.3


class ControlTorqueGazebo(Node):
    def __init__(self,metodo,tipo,world,tiempo_lado=5.):
        super().__init__('hartech_control_dinamico_esfuerzo',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.metodo=metodo
        self.tipo=tipo
        self.world=world
        self.control=crear_control(modo='pdg' if metodo in ('pdg','pidg') else 'ctc',
                                   integral=metodo in ('pidg','ctc_i'))
        self.plan=plan_completo(tipo,DT,tiempo_lado)
        self.q_goal=self.plan[1][-1]
        self.t_total=self.plan[0][-1]
        self.t_base=None
        # Solo cuadrado: el avance de la referencia depende del seguimiento.
        # El generador XY/IK permanece INTACTO.
        self.square_t=0.0
        self.square_t_anterior=None
        self.square_ritmo=0.0
        self.square_error_desde=None
        self.square_pausa_desde=None
        self.square_diagnostico_anterior=-10.0
        # CSV V5: J2 e=+2.20deg y J3 e=+2.79deg mientras tau-g era
        # +0.079 / +0.062 Nm. Se refuerza SOLO seguimiento CTC del cuadrado.
        # Extra PD en Nm (no reemplaza la dinamica RNEA). Parametros
        # EXPERIMENTALES de simulacion; no son limites de motores reales.
        self.square_bias_habilitado=(tipo=='cuadrado' and metodo=='ctc')
        self.square_bias=np.zeros(6)
        self.square_bias_ki=np.array([.10,.65,.85,.12,.50,.08]) # Nm/(rad*s)
        self.square_bias_lim=np.array([.20,.75,.65,.15,.25,.08]) # Nm
        self.square_kp_torque=np.array([.10,2.8,3.6,.25,.6,.10]) # Nm/rad
        self.square_kd_torque=np.array([.08,1.0,1.2,.10,.20,.04]) # Nm*s/rad
        self.square_pd_lim=np.array([.18,.55,.55,.15,.20,.08])  # Nm
        self.square_pd_extra=np.zeros(6)
        self.square_estancado_desde=None
        self.square_estancado_aviso=None
        # Prevalidacion: no dibujar si ni siquiera sostiene HOME en 1g.
        self.square_fase='home' if self.square_bias_habilitado else 'trayectoria'
        self.square_home_ok_desde=None
        self.square_home_limite_s=18.0
        self.square_tiempo_max_s=65.0
        self.square_tiempo_rampa_inicio=None
        self.square_inicio_trayectoria=None
        self.square_aviso_home=0.0
        self.square_terminado_avisado=False
        self.q=None;self.dq=None;self.stamp_wall=None
        self.gravedad_estado='esperando';self.gravedad_resultado=None
        self.gravedad_hilo=None;self.contador_inicio=None
        self.factor_gravedad=0.0  # fraccion [0,1] de g(q) aplicada al modelo
        self.cancelar_rampa=threading.Event()
        self.gravedad_lock=threading.Lock()  # serializa cambios de gravedad y parada
        self.last_feedback=np.zeros(6)
        self.rampa_ultimo_aviso=-10.0
        self.fallo=False;self.log_anterior=-10.;self.aviso_anterior=-10.
        self.pub=self.create_publisher(Float64MultiArray,'/arm_effort_controller/commands',10)
        self.sub=self.create_subscription(JointState,'/joint_states',self.recibir,qos_profile_sensor_data)
        self.timer=self.create_timer(DT,self.ciclo)
        self.prev_time=None;self.last_tau=np.zeros(6)
        self.ultimo_csv=-10.; self.ultimo_flush=-10.; self.ultimo_estado_log=0
        salida=Path.home()/'Hartech_ws'/'resultados_lab7_gripper'
        salida.mkdir(parents=True,exist_ok=True)
        self.csv_path=salida/('torque_%s_%s_%s.csv'%(metodo,tipo,datetime.now().strftime('%Y%m%d_%H%M%S')))
        self.f=self.csv_path.open('w',newline='',encoding='utf-8')
        self.csv=csv.writer(self.f)
        self.csv.writerow(['t_s',*[f'q{i}_rad' for i in range(1,7)],
            *[f'qd{i}_rad' for i in range(1,7)],
            *[f'dq{i}_rad_s' for i in range(1,7)],
            *[f'tau{i}_Nm' for i in range(1,7)],
            *[f'g{i}_Nm' for i in range(1,7)],
            'error_q_deg','error_tcp_mm','max_dq_deg_s','saturado',
            'tiempo_referencia_s','ritmo_referencia','error_max_eje_deg','eje_peor',
            'bias_j5_Nm','fase_cuadrado',
            *[f'bias_j{i}_Nm' for i in range(1,7)],
            *[f'pd_extra_j{i}_Nm' for i in range(1,7)]])
        self.get_logger().info('=== DINAMICA HARTECH CON GRIPPER | %s | %s ==='%(metodo,tipo))
        self.get_logger().info('Modelo gripper fijo 30 mm; frecuencia 50 Hz; mundo temporal 0g.')
        self.get_logger().info('ARRANQUE V3: 0g a 1g en pasos de 2%%, primero gravedad fisica, luego torque.')
        self.get_logger().info('Tiempo de referencia %.2f s (mas comprobacion HOME para el cuadrado).'%self.t_total)
        self.get_logger().info('Torque limite provisional [Nm]: '+str(TORQUE_PERMITIDO))
        self.get_logger().info('Kp: '+str(np.round(self.control.kp,4))+' | Kd: '+str(np.round(self.control.kd,4)))
        if self.control.integral_habilitada:
            self.get_logger().warn('MEJORA INTEGRAL ACTIVADA: no es PD puro; se busca reducir error residual.')
        if tipo=='cuadrado':
            self.get_logger().info('CTC_CUADRADO_AJUSTE_CSV: primero sostener HOME en 1g, '
                                   'despues avanzar cuadrado 6x6cm.')
            if self.square_bias_habilitado:
                self.get_logger().warn('AJUSTE CSV: realimentacion Nm adicional J2/J3 + '
                                       'integral acotada; NO es CTC puro; ganancias experimentales.')
        self.get_logger().info('CSV: '+str(self.csv_path))

    def recibir(self,msg):
        try: idx=[msg.name.index(j) for j in JOINTS]
        except ValueError: return
        if len(msg.position)<=max(idx) or len(msg.velocity)<=max(idx):return
        q=np.asarray([msg.position[i] for i in idx],dtype=float)
        dq=np.asarray([msg.velocity[i] for i in idx],dtype=float)
        if np.all(np.isfinite(q)) and np.all(np.isfinite(dq)):
            self.q,self.dq,self.stamp_wall=q,dq,time.monotonic()

    def solicitar_gravedad(self):
        """Rampa conservadora SIN torque previo a activar gravedad.

        En V2 se hacía factor_gravedad=f y se esperaba 80 ms antes de
        ``gz physics``. Eso inyectaba un torque externo en un mundo 0g:
        en articulaciones ligeras puede desencadenar un movimiento brusco.

        V3 aplica PRIMERO un incremento pequeño de gravedad (2%), espera
        confirmación del CLI y SOLO DESPUÉS escala la compensación. La
        ventana temporal de error es pequeña y no hay torque anticipado.
        Todo cambio de gravedad es excluyente con la parada de emergencia.
        """
        try:
            if shutil.which('gz') is None:
                raise RuntimeError('No existe gz (Gazebo Classic).')
            n_pasos=50  # cada paso cambia solo 0.02g; no saltar directo a 10%.
            for paso in range(1,n_pasos+1):
                if self.cancelar_rampa.is_set(): return
                factor=paso/n_pasos
                with self.gravedad_lock:
                    if self.cancelar_rampa.is_set(): return
                    cmd=['gz','physics','-w',self.world,'-g',
                         f'0,0,{-9.81*factor:.6f}']
                    respuesta=subprocess.run(cmd,capture_output=True,text=True,timeout=12)
                    if respuesta.returncode:
                        raise RuntimeError('gz physics: '+respuesta.stderr.strip()+
                                           ' '+respuesta.stdout.strip())
                    if self.cancelar_rampa.is_set(): return
                    # IMPORTANTE: factor se actualiza DESPUÉS de cambiar
                    # la gravedad real de Gazebo, nunca antes.
                    self.factor_gravedad=factor

                if paso%5==0 or paso==1:
                    q=self.q.copy() if self.q is not None else None
                    dq=self.dq.copy() if self.dq is not None else None
                    desv=(np.rad2deg(np.max(np.abs(q-Q_HOME))) if q is not None else float('nan'))
                    vel=(np.rad2deg(np.max(np.abs(dq))) if dq is not None else float('nan'))
                    self.get_logger().info('Rampa V3: %3d%% g | desviacion HOME=%6.3f deg | '
                                           'max|dq|=%6.3f deg/s'%(round(100*factor),desv,vel))
                # Tiempo de asentamiento: el control sigue publicando a 50Hz.
                time.sleep(.18)
            if not self.cancelar_rampa.is_set():
                self.gravedad_resultado=(True,'Gravedad normal alcanzada con control activo')
        except Exception as e:
            self.gravedad_resultado=(False,str(e))

    def detener_seguro(self,motivo):
        if self.fallo:return
        self.fallo=True
        self.cancelar_rampa.set()
        self.get_logger().error(motivo)
        self.get_logger().error('PAUSA O CIERRA GAZEBO. El ultimo torque no debe quedar aplicado sin control.')
        # No se puede garantizar un frenado seguro solo enviando cero esfuerzo.
        # Desactivar gravedad de forma asíncrona evita caida libre al dejar de regular.
        def tarea():
            # Evita la carrera V2: un hilo subiendo gravedad y otro poniendola en 0.
            with self.gravedad_lock:
                try:
                    respuesta=subprocess.run(
                        ['gz','physics','-w',self.world,'-g','0,0,0'],
                        timeout=8,capture_output=True,text=True)
                    if respuesta.returncode == 0:
                        self.factor_gravedad=0.
                    else:
                        self.get_logger().error('No se pudo restaurar 0g; pausar Gazebo.')
                except Exception as ex:
                    self.get_logger().error('No se pudo restaurar 0g: %s. Pausar Gazebo.'%ex)
        threading.Thread(target=tarea,daemon=True).start()
        self.pub.publish(Float64MultiArray(data=[0.]*6))
        self.f.flush()

    def ciclo(self):
        now=time.monotonic()
        if self.q is None:
            if now-self.ultimo_estado_log>2:
                self.get_logger().info('Esperando /joint_states ...')
                self.ultimo_estado_log=now
            return
        if self.pub.get_subscription_count()==0:
            if now-self.ultimo_estado_log>2:
                self.get_logger().info('Esperando /arm_effort_controller/commands ...')
                self.ultimo_estado_log=now
            return
        if self.fallo:return
        if self.stamp_wall is None or now-self.stamp_wall>MAX_AGE:
            self.detener_seguro('Estado articular obsoleto; revisar reloj /joint_states.')
            return
        q,dq=self.q.copy(),self.dq.copy()
        if np.any(q<Q_MIN-.035) or np.any(q>Q_MAX+.035) or np.max(np.abs(dq))>1.2:
            self.detener_seguro('Limite articular o velocidad excesiva.');return
        e_home=np.max(np.abs(q-Q_HOME))
        vel_max=np.max(np.abs(dq))

        # En la rampa, no se permite que el robot escape de HOME.
        if self.gravedad_estado!='activo':
            if e_home>np.deg2rad(8.):
                self.detener_seguro('Arranque: desviacion de HOME >8 grados; verificar g(q) y modelo.')
                return
            if self.gravedad_estado=='rampa' and vel_max>.6:
                self.detener_seguro('Arranque: velocidad excesiva durante rampa de gravedad.')
                return

        t_now=self.get_clock().now().nanoseconds/1e9
        dt=DT if self.prev_time is None else float(np.clip(t_now-self.prev_time,.001,.04))
        self.prev_time=t_now

        if self.gravedad_estado=='activo':
            # Tiempo de trayectoria independiente del tiempo invertido en HOME.
            t=max(0.,t_now-self.t_base)
            if self.tipo=='cuadrado' and self.square_bias_habilitado:
                if self.square_fase=='home':
                    # Mantener q_HOME hasta conseguir error y velocidad pequenos.
                    # Nunca empezar a dibujar arrastrando un error de 1-2 grados.
                    qr=Q_HOME.copy()
                    dqr=np.zeros(6); ddqr=np.zeros(6)
                    ehome_deg=float(np.rad2deg(np.max(np.abs(q-Q_HOME))))
                    velhome_deg=float(np.rad2deg(np.max(np.abs(dq))))
                    self.square_ritmo=0.0
                    if ehome_deg<0.85 and velhome_deg<0.50:
                        if self.square_home_ok_desde is None:
                            self.square_home_ok_desde=t_now
                        elif t_now-self.square_home_ok_desde>=1.25:
                            self.square_fase='trayectoria'
                            self.square_inicio_trayectoria=t_now
                            # No reiniciar t_base: t_s del CSV debe ser monotono.
                            self.square_t_anterior=t_now
                            self.square_t=0.0
                            self.square_estancado_desde=None
                            self.get_logger().info('HOME VERIFICADO a 1g: error max %.2fdeg, '
                                'vel max %.2fdeg/s. INICIANDO referencia XY.'%(ehome_deg,velhome_deg))
                    else:
                        self.square_home_ok_desde=None
                    if t>self.square_home_limite_s:
                        self.detener_seguro('CTC CSV: HOME NO CONVERGE en %.0fs; '
                            'eje mas desviado J%d=%.2fdeg, vel=%.2fdeg/s; '
                            'no iniciar cuadrado.'%(self.square_home_limite_s,
                                int(np.argmax(np.abs(q-Q_HOME)))+1,ehome_deg,velhome_deg))
                        return
                else:
                    dts=0.0 if self.square_t_anterior is None else float(np.clip(
                        t_now-self.square_t_anterior,0.,.04))
                    self.square_t_anterior=t_now
                    qr_pre,_,_=evaluar_referencia(self.square_t,self.plan)
                    e_pre=float(np.rad2deg(np.max(np.abs(qr_pre-q))))
                    # Avance continuo, salvo cuando los errores exceden 4deg.
                    if e_pre>=4.0: ritmo_objetivo=0.0
                    elif e_pre>=3.0: ritmo_objetivo=0.35
                    elif e_pre>=2.0: ritmo_objetivo=0.65
                    elif e_pre>=1.0: ritmo_objetivo=0.85
                    else: ritmo_objetivo=1.0
                    ritmo_anterior=self.square_ritmo
                    if dts>1.e-8:
                        delta=float(np.clip(ritmo_objetivo-ritmo_anterior,
                                    -1.5*dts,1.0*dts))
                        self.square_ritmo=float(np.clip(ritmo_anterior+delta,0.,1.))
                        aceleracion_ritmo=delta/dts
                    else:
                        aceleracion_ritmo=0.0
                    self.square_t=min(self.t_total,self.square_t+self.square_ritmo*dts)
                    qr,dq_nom,ddq_nom=evaluar_referencia(self.square_t,self.plan)
                    # Regla de la cadena; velocidad y aceleracion SI deben
                    # reescalarse cuando se cambia el ritmo de la referencia.
                    dqr=self.square_ritmo*dq_nom
                    ddqr=self.square_ritmo**2*ddq_nom+aceleracion_ritmo*dq_nom
                    t_mov=t_now-self.square_inicio_trayectoria if self.square_inicio_trayectoria is not None else 0.0
                    if t_mov>self.square_tiempo_max_s and self.square_t<self.t_total-.1:
                        self.detener_seguro('CTC CSV: cuadrado inconcluso en %.0fs; '
                            'ref %.1f/%.1fs, error de seguimiento elevado.'%(
                            self.square_tiempo_max_s,self.square_t,self.t_total))
                        return
            elif self.tipo=='cuadrado':
                # Otro metodo: conservar la referencia original sin ajuste CTC.
                qr,dqr,ddqr=evaluar_referencia(t,self.plan)
            else:
                qr,dqr,ddqr=evaluar_referencia(t,self.plan)
        else:
            t=0.
            qr=Q_HOME.copy()
            dqr=np.zeros(6);ddqr=np.zeros(6)

        # Protecciones SOLO despues de validar HOME (y antes de enviar torque).
        if self.square_bias_habilitado and self.gravedad_estado=='activo':
            e_joint_deg=np.abs(np.rad2deg(qr-q))
            e_max=float(np.max(e_joint_deg))
            eje=int(np.argmax(e_joint_deg))+1
            ec_pre=1000*np.linalg.norm(fkine_gripper(qr)[:3,3]-fkine_gripper(q)[:3,3])
            if e_max>10.0 or ec_pre>70.:
                self.detener_seguro('CTC CSV: DESVIACION GRAVE J%d=%.1fdeg, TCP=%.1fmm.'%(
                                    eje,e_max,ec_pre))
                return
            if self.square_fase=='trayectoria':
                seguimiento_malo=e_max>6.0 or ec_pre>35.
                if seguimiento_malo:
                    if self.square_error_desde is None:self.square_error_desde=t_now
                    if t_now-self.square_error_desde>0.8:
                        self.detener_seguro('CTC CSV: DESVIACION PERSISTENTE J%d=%.1fdeg '
                            'TCP=%.1fmm; estudiar q,dq,tau,g en CSV.'%(eje,e_max,ec_pre))
                        return
                else:
                    self.square_error_desde=None
                if e_max>3.5 or self.square_ritmo<.10:
                    if self.square_estancado_desde is None:
                        self.square_estancado_desde=t_now
                    elif t_now-self.square_estancado_desde>7.0:
                        self.detener_seguro('CTC CSV: ESTANCAMIENTO 7s, J%d=%.2fdeg '
                            'ref=%.1f/%.1fs; deteniendo.'%(
                            eje,e_max,self.square_t,self.t_total))
                        return
                else:
                    self.square_estancado_desde=None

        try:
            tau_nominal,aux=self.control.calcular(q,dq,qr,dqr,ddqr,dt=dt)
        except Exception as e:
            self.detener_seguro('Fallo de calculo de torque: '+str(e));return

        # Separar la compensacion gravitacional del esfuerzo de realimentacion.
        # En 0g: esfuerzo de gravedad = 0; en rampa: f*g(q); en 1g: g(q).
        # NO limitar la tasa de cambio de g(q): antes ese limitador causaba
        # retraso en aplicar los torques para sostener el brazo al activar 1g.
        g=aux['g']
        # NO usar el torque ya recortado: aux['raw'] es el torque RNEA
        # sin saturar, y así la separación (raw - g) es físicamente válida.
        feedback=aux['raw']-g
        rate=np.array([100.,100.,70.,30.,30.,15.])
        feedback=np.clip(feedback,self.last_feedback-rate*dt,self.last_feedback+rate*dt)
        self.last_feedback=feedback.copy()
        tau_pedido=feedback+self.factor_gravedad*g
        torque_limites=np.array([37.5,37.5,22.5,7.5,7.5,3.75])
        self.square_pd_extra[:]=0.
        if self.square_bias_habilitado and self.gravedad_estado=='activo':
            # Refuerzo rapido segun CSV V5: J2 (+2.20deg), J3 (+2.79deg).
            # PD DIRECTO EN TORQUE para evitar que los pequenos M(q) del
            # modelo subestimen la correccion fisica requerida en Gazebo.
            err=qr-q
            err_d=dqr-aux['dq_f']
            extra=self.square_kp_torque*err+self.square_kd_torque*err_d
            self.square_pd_extra=np.clip(extra,-self.square_pd_lim,self.square_pd_lim)
            # Integral lenta para discrepancias constantes; activa tambien
            # mientras sostiene HOME. No aplicar en 0g ni en la rampa.
            sesgo_previo=self.square_bias.copy()
            e_limitado=np.clip(err,-np.deg2rad(10),np.deg2rad(10))
            self.square_bias=np.clip(self.square_bias+
                self.square_bias_ki*e_limitado*dt,
                -self.square_bias_lim,self.square_bias_lim)
            tau_candidato=tau_pedido+self.square_pd_extra+self.square_bias
            # Anti-windup: deshacer incremento integral solo donde ya
            # se sobrepasa el limite y el incremento empeora la saturacion.
            sat_tmp=np.abs(tau_candidato)>torque_limites
            empeora=sat_tmp & (np.sign(self.square_bias-sesgo_previo)==
                                 np.sign(tau_candidato))
            self.square_bias[empeora]=sesgo_previo[empeora]
            tau_pedido=tau_pedido+self.square_pd_extra+self.square_bias
        saturado=bool(aux['sat'] or np.any(np.abs(tau_pedido)>torque_limites))
        tau=np.clip(tau_pedido,-torque_limites,torque_limites)
        self.last_tau=tau.copy()
        self.pub.publish(Float64MultiArray(data=tau.tolist()))

        if self.gravedad_estado=='esperando':
            if vel_max>.08:
                self.contador_inicio=None
                return
            if self.contador_inicio is None:self.contador_inicio=now
            if now-self.contador_inicio<.50:return
            self.gravedad_estado='rampa'
            self.gravedad_hilo=threading.Thread(target=self.solicitar_gravedad,daemon=True)
            self.gravedad_hilo.start()
            self.get_logger().info('HOME estable. Iniciando rampa de gravedad CON torques activos.')
            return
        if self.gravedad_estado=='rampa':
            if self.gravedad_resultado is None:
                if now-self.rampa_ultimo_aviso>2.:
                    self.rampa_ultimo_aviso=now
                    self.get_logger().info('Arranque: gravedad ~%.0f%%, desviacion HOME %.2f deg, '
                                           'max_dq %.2f deg/s'%(100*self.factor_gravedad,
                                            np.rad2deg(e_home),np.rad2deg(vel_max)))
                return
            ok,msj=self.gravedad_resultado
            if not ok:
                self.detener_seguro('Error habilitando gravedad: '+msj);return
            self.gravedad_estado='activo'
            self.factor_gravedad=1.0
            self.t_base=t_now
            # La ley de control se mantiene; reiniciar la integral de la etapa
            # de arranque para comparar trayectorias desde la misma condicion.
            self.control.reiniciar()
            self.square_bias[:]=0.
            self.square_pd_extra[:]=0.
            self.square_estancado_desde=None
            self.square_fase='home' if self.square_bias_habilitado else 'trayectoria'
            self.square_home_ok_desde=None
            self.get_logger().info(msj+'; INICIANDO trayectoria dinamica.')
            return

        eq=np.rad2deg(np.linalg.norm(qr-q))
        ec=1000*np.linalg.norm(fkine_gripper(qr)[:3,3]-fkine_gripper(q)[:3,3])
        if t-self.ultimo_csv>=.1:
            e_vec=np.abs(np.rad2deg(qr-q))
            self.csv.writerow([f'{t:.4f}',*q,*qr,*dq,*tau,*g,
                               f'{eq:.5f}',f'{ec:.5f}',f'{np.rad2deg(vel_max):.4f}',
                               int(saturado),f'{self.square_t:.4f}' if self.tipo=='cuadrado' else f'{t:.4f}',
                               f'{self.square_ritmo:.3f}' if self.tipo=='cuadrado' else '1.0',
                               f'{np.max(e_vec):.3f}',int(np.argmax(e_vec))+1,
                               f'{self.square_bias[4]:.5f}',self.square_fase,
                                *self.square_bias,*self.square_pd_extra])
            self.ultimo_csv=t
        if t-self.ultimo_flush>=2.:
            self.f.flush();self.ultimo_flush=t
        if t-self.log_anterior>=1.:
            self.log_anterior=t
            if self.tipo=='cuadrado':
                self.get_logger().info('t=%5.1fs | ref=%5.1fs/%4.0fs | ritmo=%.2f | '
                    'fase=%s | J%d=%.2fdeg | e_q=%.2fdeg | e_TCP=%.1fmm | '
                    'PD_J3=%+.3fNm | bias_J3=%+.3fNm | sat=%d'%
                    (t,self.square_t,self.t_total,self.square_ritmo,self.square_fase,
                     int(np.argmax(np.abs(qr-q)))+1,
                     np.max(np.abs(np.rad2deg(qr-q))),eq,ec,
                     self.square_pd_extra[2],self.square_bias[2],int(saturado)))
            else:
                self.get_logger().info('t=%5.1fs | e_q=%6.3fdeg | e_TCP=%7.2fmm | '
                    'max_dq=%5.2fdeg/s | max_tau=%6.2fNm | sat=%d'%
                    (t,eq,ec,np.rad2deg(vel_max),max(abs(tau)),int(saturado)))
        # No avisar error de estacionario antes de terminar la referencia adaptativa.
        if ((self.tipo!='cuadrado' or self.square_t>=self.t_total) and
            t>self.t_total+6 and (eq>2. or ec>10.) and t-self.aviso_anterior>8):
            self.aviso_anterior=t
            self.get_logger().warn('Error residual elevado; comparar CSV y modelo.')
        if (self.tipo!='cuadrado' or self.square_t>=self.t_total) and t>self.t_total+10 and eq<1. and ec<5. and vel_max<.02:
            if t-self.aviso_anterior>8:
                self.aviso_anterior=t
                self.get_logger().info('REGULACION NUMERICA OK: error <1deg y TCP <5mm.')

    def cerrar(self):
        self.cancelar_rampa.set()
        try:
            self.f.flush();self.f.close()
            self.get_logger().info('CSV guardado: '+str(self.csv_path))
        except Exception:pass


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--metodo',choices=['pdg','pidg','ctc','ctc_i'],default='pidg')
    p.add_argument('--tipo',choices=['regulacion','trayectoria','cuadrado'],default='regulacion')
    p.add_argument('--gazebo-world',default='robot_lab')
    p.add_argument('--tiempo-lado',type=float,default=5.)
    args=p.parse_args(remove_ros_args()[1:])
    rclpy.init();node=None
    try:
        node=ControlTorqueGazebo(args.metodo,args.tipo,args.gazebo_world,args.tiempo_lado)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cerrar();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
