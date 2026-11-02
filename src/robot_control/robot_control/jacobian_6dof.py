"""Jacobiano geométrico espacial (base_link), 6x6, del frame tool0.

J[:3, :] relaciona qdot [rad/s] con velocidad lineal [m/s].
J[3:, :] relaciona qdot [rad/s] con velocidad angular [rad/s].
Cálculo numérico central a partir de la FK verificada del HARTECH.
"""
import numpy as np
from fkine_functions import fkine_hartech


def jacobian_6dof(q, h=1e-5):
    """Devuelve J=[Jv; Jw] expresado en los ejes de base_link."""
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError('q debe contener seis ángulos articulares finitos [rad].')
    if not np.isfinite(h) or h <= 0:
        raise ValueError('h debe ser positivo y finito.')
    R = fkine_hartech(q)[:3, :3]
    J = np.empty((6, 6), dtype=float)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        Tp = fkine_hartech(q + dq)
        Tm = fkine_hartech(q - dq)
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2*h)
        # dR/dq_i * R^T produce la velocidad angular espacial.
        Omega = ((Tp[:3, :3] - Tm[:3, :3]) / (2*h)) @ R.T
        Omega = 0.5 * (Omega - Omega.T)  # eliminar ruido de simetría
        J[3:, i] = [Omega[2, 1], Omega[0, 2], Omega[1, 0]]
    return J


def singular_values_scaled(J, length=0.3):
    """Valores singulares de un Jacobiano escalado a unidades comparables.

    Los umbrales dependen de la longitud característica y de la tarea.
    No constituyen un certificado de seguridad o de colisión.
    """
    if length <= 0 or not np.isfinite(length):
        raise ValueError('La longitud característica debe ser positiva.')
    Jscaled = np.asarray(J, dtype=float).copy()
    Jscaled[3:, :] *= length
    return np.linalg.svd(Jscaled, compute_uv=False)
