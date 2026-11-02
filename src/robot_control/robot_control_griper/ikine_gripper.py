#!/usr/bin/env python3
"""IK de posición XYZ del TCP HARTECH, con límites articulares (RViz)."""
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_control_griper.fkine_gripper import fkine_gripper

JOINTS = [f'joint_{i}' for i in range(1, 7)]
EPS_POS = 0.001  # 1 mm
MAX_ITER = 250


def fk_pos(q):
    return fkine_gripper(q)[:3, 3]


def jacobian_position_gripper(q, h=1e-5):
    """Jacobiano 3x6 del TCP mediante diferencias centrales."""
    J = np.empty((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        J[:, i] = (fk_pos(q + dq) - fk_pos(q - dq)) / (2.0 * h)
    return J


def resolver_desde_semilla(xdes, semilla, qmin, qmax):
    """Newton amortiguado con límites, búsqueda de paso y control de convergencia."""
    q = np.clip(np.asarray(semilla, dtype=float).copy(), qmin, qmax)
    historial = []
    mejor_error, estancado = float('inf'), 0
    for _ in range(MAX_ITER):
        e = xdes - fk_pos(q)
        err = float(np.linalg.norm(e))
        historial.append(err)
        if err < EPS_POS:
            return q, historial, True
        if err < mejor_error - 1e-8:
            mejor_error, estancado = err, 0
        else:
            estancado += 1
            if estancado >= 25:
                break
        J = jacobian_position_gripper(q)
        lam = 0.015 if err < 0.05 else 0.04
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), e)
        except np.linalg.LinAlgError:
            break
        if not np.all(np.isfinite(dq)):
            break
        magnitud = np.linalg.norm(dq)
        if magnitud > 0.2:
            dq *= 0.2 / magnitud
        aceptado = False
        for factor in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            candidato = np.clip(q + factor * dq, qmin, qmax)
            if np.linalg.norm(xdes - fk_pos(candidato)) < err - 1e-10:
                q, aceptado = candidato, True
                break
        if not aceptado:
            break
    historial.append(float(np.linalg.norm(xdes - fk_pos(q))))
    return q, historial, historial[-1] < EPS_POS


def ikine_gripper(xdes, q0):
    """Resuelve el TCP; retorna (q_sol, errores), o (None, errores) si falla."""
    xdes = np.asarray(xdes, dtype=float)
    q0 = np.asarray(q0, dtype=float)
    if xdes.shape != (3,) or q0.shape != (6,) or not (np.all(np.isfinite(xdes)) and np.all(np.isfinite(q0))):
        raise ValueError('Se requieren XYZ (3) y q inicial (6), todos finitos.')

    ruta = Path(get_package_share_directory('robot_description'))
    config = yaml.safe_load((ruta / 'config/robot_parameters.yaml').read_text())
    qmin = np.array([float(config['joints'][j]['lower']) for j in JOINTS])
    qmax = np.array([float(config['joints'][j]['upper']) for j in JOINTS])
    qhome = np.array([float(config['joints'][j]['home']) for j in JOINTS])
    if (not np.all(np.isfinite(qmin)) or not np.all(np.isfinite(qmax))
            or np.any(qmin >= qmax)):
        raise ValueError('Límites articulares inválidos en robot_parameters.yaml.')

    semillas = [q0.copy(), qhome.copy(), np.zeros(6), np.deg2rad([0, -90, 0, 0, 0, 0])]
    for delta in (-0.6, 0.6):
        for i in (0, 1, 2, 4):
            s = q0.copy()
            s[i] += delta
            semillas.append(s)

    candidatos = []
    mejor_fallido = None
    for semilla in semillas:
        qsol, ee, ok = resolver_desde_semilla(xdes, semilla, qmin, qmax)
        err = float(np.linalg.norm(xdes - fk_pos(qsol)))
        if ok:
            distancia = float(np.linalg.norm(qsol - q0))
            candidatos.append((distancia, err, qsol, ee))
        elif mejor_fallido is None or err < mejor_fallido[0]:
            mejor_fallido = (err, ee)
    if not candidatos:
        print('Cinemática inversa TCP: no se encontró solución dentro de límites y 1 mm.')
        return None, (mejor_fallido[1] if mejor_fallido else [])

    candidatos.sort(key=lambda item: (item[0], item[1]))
    _, error, qsol, ee = candidatos[0]
    print('Cinemática inversa TCP: solución válida. Error XYZ [m]:', round(error, 6))
    return qsol, ee
