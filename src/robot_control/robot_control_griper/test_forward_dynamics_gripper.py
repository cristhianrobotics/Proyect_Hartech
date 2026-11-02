#!/usr/bin/env python3
import pinocchio as pin
import numpy as np
import argparse
from robot_control_griper.modelo_dinamico_gripper import cargar_modelo_con_gripper

## Dinámica directa - ABA (NUEVO)

def main():
    parser = argparse.ArgumentParser(description='Dinámica HARTECH con gripper a apertura fija.')
    parser.add_argument('--mm', type=float, default=30.0,
                        help='Apertura fija del gripper en mm (0 a 50, predeterminado 30).')
    args = parser.parse_args()
    # ----------------------------------------------------------
    # Lectura del modelo HARTECH a partir del URDF
    # ----------------------------------------------------------
    # URDF combinado del brazo y gripper; se fijan dedos y rotor.
    model = cargar_modelo_con_gripper(args.mm)

    data = model.createData()

    np.set_printoptions(suppress=True, precision=6)

    if model.nq != 6 or model.nv != 6:
        raise ValueError("El modelo cargado no tiene 6 GDL.")

    print("\n=== DINAMICA DIRECTA HARTECH CON GRIPPER FIJO - ABA ===")
    print("Modelo: HARTECH + gripper (articulaciones del gripper bloqueadas)")
    print(f"Apertura fija del gripper: {args.mm:g} mm")
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