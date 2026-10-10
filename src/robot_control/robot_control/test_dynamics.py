#!/usr/bin/env python3
import pinocchio as pin
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


#Dinámica inversa - RNEA (Recursive Newton-Euler Algorithm)

def main():
    # ----------------------------------------------------------
    # Lectura del modelo HARTECH a partir del URDF
    # ----------------------------------------------------------
    ruta = Path(get_package_share_directory('robot_description'))
    urdf = ruta / 'urdf' / 'robot_6dof.urdf'

    if not urdf.is_file():
        raise FileNotFoundError(f"No se encontró el URDF: {urdf}")

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()

    # Evitar notación científica en los resultados
    np.set_printoptions(suppress=True, precision=3)

    # Verificar modelo de 6 GDL
    if model.nq != 6 or model.nv != 6:
        raise ValueError("El modelo cargado no tiene 6 GDL.")

    # Verificar orden articular
    jnames = ['joint_1', 'joint_2', 'joint_3',
              'joint_4', 'joint_5', 'joint_6']

    orden = list(model.names)[1:]

    if orden != jnames:
        raise ValueError(f"Orden articular inesperado: {orden}")

    print("\n=== DINAMICA DEL ROBOT HARTECH - 6 GDL ===")
    print("Modelo URDF:", urdf)
    print("Grados de libertad:", model.nv)
    print("Articulaciones:", orden)

    # ----------------------------------------------------------
    # Configuración articular - HOME HARTECH
    # ----------------------------------------------------------
    q = np.deg2rad([0, -50, 80, 0, -30, 0])

    # Velocidad articular [rad/s]
    dq = np.array([0.1, 0.2, 0.1, 0.1, 0.1, 0.1], dtype=float)

    # Aceleración articular [rad/s²]
    ddq = np.array([0.2, 0.1, 0.2, 0.1, 0.1, 0.1], dtype=float)


    print("\nConfiguración articular [deg]:")
    print(np.rad2deg(q))

    print("\nVelocidad articular [rad/s]:")
    print(dq)

    print("\nAceleración articular [rad/s²]:")
    print(ddq)

    # ----------------------------------------------------------
    # Parte 1: Cálculo de los efectos dinámicos
    # ----------------------------------------------------------

    # g(q): vector de torques gravitacionales
    g = pin.computeGeneralizedGravity(model, data, q).copy()

    # C(q,dq): matriz de Coriolis y centrífuga
    C = pin.computeCoriolisMatrix(model, data, q, dq).copy()

    # c(q,dq): vector de efectos de Coriolis y centrífugos
    c = C @ dq

    # M(q): matriz de inercia articular
    M = pin.crba(model, data, q).copy()

    # Completar la parte inferior de la matriz simétrica
    M = np.triu(M) + np.triu(M, 1).T

    print("\n--------------------------------------")
    print("VECTOR DE GRAVEDAD g(q) [N.m]:")
    print(g)

    print("\nVECTOR DE CORIOLIS c(q,dq) [N.m]:")
    print(c)

    print("\nMATRIZ DE INERCIA M(q) [kg.m²]:")
    print(M)

    # ----------------------------------------------------------
    # Parte 2: Cálculo de torques por dinámica inversa
    # ----------------------------------------------------------

    # Ecuación dinámica
    tau_recon = M @ ddq + c + g

    print("\n--------------------------------------")
    print("TORQUES CALCULADOS [N.m]:")
    print(tau_recon)

    # Cálculo directo mediante RNEA
    tau_rnea = pin.rnea(model, data, q, dq, ddq).copy()

    print("\nTORQUES MEDIANTE RNEA [N.m]:")
    print(tau_rnea)

    # ----------------------------------------------------------
    # Parte 3: Verificación de resultados
    # ----------------------------------------------------------

    error = np.linalg.norm(tau_recon - tau_rnea)

    print("\n--------------------------------------")
    print("Error entre ambos métodos:", error)

    if np.allclose(tau_recon, tau_rnea, atol=1e-8, rtol=1e-8):
        print("Verificación correcta: los torques coinciden.")
    else:
        print("ADVERTENCIA: los resultados no coinciden.")

if __name__ == '__main__':
    main()