#!/usr/bin/env python3
import pinocchio as pin
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

## Dinámica directa - ABA (NUEVO)

def main():
    # ----------------------------------------------------------
    # Lectura del modelo HARTECH a partir del URDF
    # ----------------------------------------------------------
    ruta = Path(get_package_share_directory('robot_description'))
    urdf = ruta / 'urdf' / 'robot_6dof.urdf'

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()

    np.set_printoptions(suppress=True, precision=6)

    if model.nq != 6 or model.nv != 6:
        raise ValueError("El modelo cargado no tiene 6 GDL.")

    print("\n=== DINAMICA DIRECTA HARTECH - ABA ===")
    print("Modelo URDF:", urdf)
    print("Grados de libertad:", model.nv)

    # ----------------------------------------------------------
    # Configuración articular - HOME HARTECH
    # ----------------------------------------------------------
    q = np.deg2rad([0, -50, 80, 0, -30, 0])

    # Velocidad articular [rad/s]
    dq = np.array([0.1, 0.2, 0.1, 0.1, 0.1, 0.1], dtype=float)

    # Aceleración de referencia [rad/s²]
    ddq = np.array([0.2, 0.1, 0.2, 0.1, 0.1, 0.1], dtype=float)

    print("\nConfiguración articular [deg]:")
    print(np.rad2deg(q))

    print("\nVelocidad articular [rad/s]:")
    print(dq)

    # ----------------------------------------------------------
    # Parte 1: Obtener torques de referencia mediante RNEA
    # ----------------------------------------------------------
    tau = pin.rnea(model, data, q, dq, ddq).copy()

    print("\n--------------------------------------")
    print("TORQUES ARTICULARES [N.m]:")
    print(tau)

    # ----------------------------------------------------------
    # Parte 2: Dinámica directa mediante ABA
    # ----------------------------------------------------------
    ddq_aba = pin.aba(model, data, q, dq, tau).copy()

    print("\n--------------------------------------")
    print("ACELERACIONES ORIGINALES [rad/s²]:")
    print(ddq)

    print("\nACELERACIONES CALCULADAS MEDIANTE ABA [rad/s²]:")
    print(ddq_aba)

    # ----------------------------------------------------------
    # Parte 3: Verificación de resultados
    # ----------------------------------------------------------
    error = np.linalg.norm(ddq - ddq_aba)

    print("\n--------------------------------------")
    print("Error entre aceleraciones:", error)

    if np.allclose(ddq, ddq_aba, atol=1e-8, rtol=1e-8):
        print("Verificación correcta: las aceleraciones coinciden.")
    else:
        print("ADVERTENCIA: las aceleraciones no coinciden.")

if __name__ == '__main__':
    main()