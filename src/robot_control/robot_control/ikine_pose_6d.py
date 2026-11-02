#!/usr/bin/env python3
"""IK geométrica 6D de HARTECH: posición XYZ + orientación RPY.

Una sola implementación sirve para tool0 y para hartech_tcp; se reciben
la cinemática directa y el Jacobiano geométrico 6x6 correspondientes.
No modifica las funciones DH, el Jacobiano ni las IK de posición existentes.
"""
import numpy as np

EPS_POS = 0.001                   # m (1 mm)
EPS_ANG = np.deg2rad(0.5)         # rad (0.5 grados)
LONGITUD_ESCALA = 0.30           # m/rad, para equilibrar traslación y rotación
MAX_ITER = 220
JOINTS = [f'joint_{i}' for i in range(1, 7)]


def rpy_a_rotacion(rpy_grados):
    """Roll, pitch, yaw [grados], convención R=Rz(yaw)Ry(pitch)Rx(roll)."""
    rpy = np.asarray(rpy_grados, dtype=float)
    if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
        raise ValueError('RPY debe contener 3 ángulos finitos en grados.')
    roll, pitch, yaw = np.deg2rad(rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotacion_a_rpy(R):
    """RPY [grados] de una matriz de rotación, con manejo de pitch=+-90°."""
    R = np.asarray(R, dtype=float)
    cp = float(np.hypot(R[0, 0], R[1, 0]))
    pitch = np.arctan2(-R[2, 0], cp)
    if cp > 1e-8:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0  # RPY no es único en pitch=+-90°, elegir roll=0.
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return np.rad2deg([roll, pitch, yaw])


def rotacion_a_cuaternion_xyzw(R):
    """Cuaternión robusto [x,y,z,w], incluso para rotaciones de 180°."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2])/s, (R[0, 2] - R[2, 0])/s,
             (R[1, 0] - R[0, 1])/s, s/4.0]
    else:
        k = int(np.argmax(np.diag(R)))
        if k == 0:
            s = 2.0 * np.sqrt(max(1e-15, 1.0 + R[0, 0] - R[1, 1] - R[2, 2]))
            q = [s/4.0, (R[0, 1] + R[1, 0])/s,
                 (R[0, 2] + R[2, 0])/s, (R[2, 1] - R[1, 2])/s]
        elif k == 1:
            s = 2.0 * np.sqrt(max(1e-15, 1.0 + R[1, 1] - R[0, 0] - R[2, 2]))
            q = [(R[0, 1] + R[1, 0])/s, s/4.0,
                 (R[1, 2] + R[2, 1])/s, (R[0, 2] - R[2, 0])/s]
        else:
            s = 2.0 * np.sqrt(max(1e-15, 1.0 + R[2, 2] - R[0, 0] - R[1, 1]))
            q = [(R[0, 2] + R[2, 0])/s, (R[1, 2] + R[2, 1])/s,
                 s/4.0, (R[1, 0] - R[0, 1])/s]
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q  # usar la rotación de menor ángulo
    return q


def error_orientacion(R_deseada, R_actual):
    """Vector log(R_des @ R_act.T), expresado en ejes base_link [rad]."""
    qerr = rotacion_a_cuaternion_xyzw(R_deseada @ R_actual.T)
    v, w = qerr[:3], qerr[3]
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return 2.0*v
    return (2.0*np.arctan2(n, w)/n)*v


def errores_pose(T_des, T_actual):
    ep = T_des[:3, 3] - T_actual[:3, 3]
    er = error_orientacion(T_des[:3, :3], T_actual[:3, :3])
    return ep, er


def solucionar_desde_semilla(T_des, semilla, qmin, qmax, fk, jac):
    """Newton amortiguado, paso acotado y búsqueda de mejora con límites."""
    q = np.clip(np.asarray(semilla, dtype=float), qmin, qmax).copy()
    historial = []
    for _ in range(MAX_ITER):
        ep, er = errores_pose(T_des, fk(q))
        epos, eang = np.linalg.norm(ep), np.linalg.norm(er)
        historial.append(float(np.hypot(epos, LONGITUD_ESCALA*eang)))
        if epos < EPS_POS and eang < EPS_ANG:
            return q, historial, True, epos, eang
        e = np.concatenate((ep, LONGITUD_ESCALA*er))
        J = np.asarray(jac(q), dtype=float).copy()
        J[3:, :] *= LONGITUD_ESCALA
        # DLS, estable cerca de configuraciones singulares.
        amort = 0.02 if np.linalg.norm(e) < 0.10 else 0.06
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + amort**2 * np.eye(6), e)
        except np.linalg.LinAlgError:
            break
        if not np.all(np.isfinite(dq)):
            break
        norma = np.linalg.norm(dq)
        if norma > 0.20:
            dq *= 0.20/norma
        actual = float(np.linalg.norm(e))
        mejoro = False
        for factor in (1., 0.5, 0.25, 0.125, 0.0625, 0.03125):
            q_prueba = np.clip(q + factor*dq, qmin, qmax)
            ep2, er2 = errores_pose(T_des, fk(q_prueba))
            costo = float(np.hypot(np.linalg.norm(ep2),
                                   LONGITUD_ESCALA*np.linalg.norm(er2)))
            if costo < actual - 1e-10:
                q, mejoro = q_prueba, True
                break
        if not mejoro:
            break
    ep, er = errores_pose(T_des, fk(q))
    return q, historial, False, float(np.linalg.norm(ep)), float(np.linalg.norm(er))


def ik_pose_6d(T_des, qactual, qmin, qmax, qhome, fk, jac):
    """Devuelve (qsol, historial, errores) o (None, historial, errores).

    T_des es una transformación homogénea base_link->tool0/TCP.
    No se publica una solución a menos que cumpla ambas tolerancias.
    """
    T_des = np.asarray(T_des, dtype=float)
    qactual, qmin, qmax, qhome = (np.asarray(v, dtype=float) for v in
                                  (qactual, qmin, qmax, qhome))
    if (T_des.shape != (4, 4) or not np.all(np.isfinite(T_des)) or
        any(v.shape != (6,) or not np.all(np.isfinite(v))
            for v in (qactual, qmin, qmax, qhome)) or
        np.any(qmin >= qmax)):
        raise ValueError('Pose, semilla o límites articulares inválidos.')
    semillas = [qactual, qhome, np.zeros(6),
                np.deg2rad([30, -45, 60, 0, -20, 0]),
                np.deg2rad([0, -90, 0, 0, 0, 0])]
    for i in (1, 2, 3, 4):
        for signo in (-1, 1):
            s = qactual.copy()
            s[i] += signo*0.40
            semillas.append(s)
    soluciones = []
    mejor_fallo = None
    for s in semillas:
        q, historial, ok, epos, eang = solucionar_desde_semilla(
            T_des, s, qmin, qmax, fk, jac)
        if ok:
            distancia = float(np.linalg.norm(q - qactual))
            soluciones.append((distancia, epos+LONGITUD_ESCALA*eang,
                               q.copy(), historial, (epos, eang)))
        elif mejor_fallo is None or epos+LONGITUD_ESCALA*eang < mejor_fallo[0]:
            mejor_fallo = (epos+LONGITUD_ESCALA*eang, historial, (epos, eang))
    if not soluciones:
        return None, (mejor_fallo[1] if mejor_fallo else []), (
            mejor_fallo[2] if mejor_fallo else (float('inf'), float('inf')))
    soluciones.sort(key=lambda v: (v[0], v[1]))
    _, _, qsol, historial, errores = soluciones[0]
    return qsol, historial, errores
