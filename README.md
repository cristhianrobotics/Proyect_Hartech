# Proyect_Hartech
Development of a 6-Development and implementation of a 6-DOF robotic manipulator using ROS 2, including modeling, simulation, kinematics, motion control and embedded system integration - hardware.


## Guía de ejecución — HARTECH (ROS 2 Humble)

Esta sección reúne los comandos para **clonar, compilar y ejecutar** los paquetes `robot_description` y `robot_control` del manipulador HARTECH de 6 GDL.

**Requisitos previos:** se asume que el equipo ya tiene **Ubuntu 22.04, ROS 2 Humble** y las dependencias del proyecto; para las pruebas en Gazebo, **Gazebo Classic 11** y `gazebo_ros2_control`; y para dinámica, **Pinocchio**. Esta guía no cubre su instalación.

> **Ejecutar cada modalidad por separado.** Para cambiar de prueba, finalizar los procesos anteriores con `Ctrl + C`. No mantener abierto RViz con sliders al ejecutar los nodos de cinemática, ni enviar órdenes al controlador de Gazebo desde dos nodos a la vez.

### 1. Clonar el repositorio y preparar el workspace

**Terminal 1 — Primera vez:**

```bash
mkdir -p ~/Hartech_ws
cd ~/Hartech_ws
git clone https://github.com/cristhianrobotics/Proyect_Hartech.git .
```

El **punto final (`.`)** clona directamente dentro de `~/Hartech_ws`; para utilizarlo, esa carpeta debe estar vacía.

La estructura resultante es:

```text
Hartech_ws/
├── src/
│   ├── robot_description/
│   └── robot_control/
├── README.md
└── .gitignore
```

**Terminal 1 — Compilar una vez:**

```bash
cd ~/Hartech_ws
colcon build
source install/setup.bash
```

**En cada terminal nueva**, antes de lanzar los ejemplos, ejecutar:

```bash
cd ~/Hartech_ws
source install/setup.bash
```

### 2. RViz básico — Visualización del robot con sliders

**Terminal 1 — `robot_description`:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch robot_description display.launch.py
```

Se abre RViz con `joint_state_publisher_gui`. Las seis articulaciones se manipulan directamente con los sliders. El modelo inicia en **HOME**.

Para abrirlo en la configuración **horizontal (todos los ángulos en cero)**, usar en su lugar:

```bash
ros2 launch robot_description display.launch.py pose:=zero
```

**No se necesita una segunda terminal.**

### 3. Gazebo básico — Simulación y trayectorias predefinidas

**Terminal 1 — Gazebo y RViz:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

**Alternativa: solo Gazebo, sin RViz:**

```bash
ros2 launch robot_description gazebo.launch.py rviz:=false
```

Esperar el mensaje `Gazebo controllers are active. Ready for trajectories.`

**Terminal 2 — Ejecutar cada movimiento por separado:**

```bash
cd ~/Hartech_ws
source install/setup.bash

# Movimiento de prueba
ros2 run robot_description send_trajectory.py --pose test --duration 5

# Regresar a HOME
ros2 run robot_description send_trajectory.py --pose home --duration 5

# Configuración horizontal (cero articular)
ros2 run robot_description send_trajectory.py --pose zero --duration 5
```

Esperar que termine cada movimiento antes de ejecutar el siguiente.

### 4. Cinemática directa en RViz — `test_forward_kinematic`

**Terminal 1 — RViz sin sliders:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch robot_description display_control.launch.py
```

**Terminal 2 — Nodo de cinemática directa:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_forward_kinematic
```

Cuando aparezca **`q[deg] >`**, ingresar **una línea de seis ángulos en grados y pulsar Enter**. Probar estas cuatro configuraciones, una por vez:

| Prueba | Valores que se escriben en `q[deg] >` |
|---|---|
| HOME | `0 -50 80 0 -30 0` |
| Horizontal | `0 0 0 0 0 0` |
| Vertical | `0 -90 0 0 0 0` |
| Otra configuración | `30 -45 60 0 -20 0` |

El nodo muestra la transformación calculada y actualiza la pose en RViz.

### 5. Cinemática inversa en RViz — `test_inverse_kinematics`

**Terminal 1 — RViz sin sliders:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch robot_description display_control.launch.py
```

**Terminal 2 — Nodo de cinemática inversa:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_inverse_kinematics
```

Cuando aparezca **`xdes[m] >`**, ingresar **una línea con X Y Z en metros y pulsar Enter**:

| Prueba | Valores que se escriben en `xdes[m] >` |
|---|---|
| HOME | `0.3674 0 0.3081` |
| Horizontal | `0.448 0 0.273` |
| Vertical | `0 0 0.721` |
| Otra posición | `0.342 0.197 0.351` |

El nodo calcula la solución articular y guarda una gráfica de convergencia `error_ikine_*.png`. **Esta implementación de RViz no restringe los ángulos durante Newton**; un objetivo puede no converger o producir una configuración fuera de los límites físicos.

### 6. Cinemática directa en Gazebo — `test_forward_gazebo`

**Terminal 1 — Iniciar solo Gazebo:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch robot_description gazebo.launch.py rviz:=false
```

Esperar que `arm_controller` esté activo.

**Terminal 2 — Nodo de cinemática directa:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_forward_gazebo
```

Cuando aparezca **`q[deg] >`**, ingresar las configuraciones una por vez:

| Prueba | Valores que se escriben en `q[deg] >` |
|---|---|
| HOME | `0 -50 80 0 -30 0` |
| Horizontal | `0 0 0 0 0 0` |
| Vertical | `0 -90 0 0 0 0` |
| Otra configuración | `30 -45 60 0 -20 0` |

El nodo envía la trayectoria a Gazebo y **calcula la duración automáticamente**; no es necesario ingresar `--duration`.

### 7. Cinemática inversa en Gazebo — `test_inverse_gazebo`

**Terminal 1 — Iniciar solo Gazebo:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch robot_description gazebo.launch.py rviz:=false
```

Esperar que `arm_controller` esté activo.

**Terminal 2 — Nodo de cinemática inversa:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_inverse_gazebo
```

Cuando aparezca **`xdes[m] >`**, ingresar una posición a la vez:

| Prueba | Valores que se escriben en `xdes[m] >` |
|---|---|
| HOME | `0.3674 0 0.3081` |
| Horizontal | `0.448 0 0.273` |
| Vertical | `0 0 0.721` |
| Otra posición | `0.342 0.197 0.351` |

El nodo busca soluciones respetando los límites articulares, envía la trayectoria y verifica la posición alcanzada. La duración también es automática. Guarda gráficas `error_ikine_gazebo_*.png`. La convergencia depende del objetivo y de la configuración actual.

> No ejecutar `test_forward_gazebo` y `test_inverse_gazebo` simultáneamente: ambos envían órdenes al mismo controlador.

### 8. Dinámica inversa — RNEA

**Una sola terminal (no requiere RViz ni Gazebo):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_dynamics
```

El programa muestra `M(q)`, `c(q,dq)`, `g(q)` y los torques obtenidos mediante RNEA.

**Importante:** no pide datos por teclado. Para repetir las dos pruebas realizadas, establecer en `src/robot_control/robot_control/test_dynamics.py`:

**Caso 1 — HOME en reposo:**

```python
q = np.deg2rad([0, -50, 80, 0, -30, 0])
dq = np.array([0, 0, 0, 0, 0, 0], dtype=float)
ddq = np.array([0, 0, 0, 0, 0, 0], dtype=float)
```

**Caso 2 — HOME con velocidades y aceleraciones:**

```python
q = np.deg2rad([0, -50, 80, 0, -30, 0])
dq = np.array([0.1, 0.2, 0.1, 0.1, 0.1, 0.1], dtype=float)
ddq = np.array([0.2, 0.1, 0.2, 0.1, 0.1, 0.1], dtype=float)
```

Guardar el archivo, ejecutar `colcon build` desde `~/Hartech_ws`, cargar `source install/setup.bash` y volver a ejecutar `ros2 run robot_control test_dynamics`.

### 9. Dinámica directa — ABA

**Una sola terminal (no requiere RViz ni Gazebo):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_forward_dynamics
```

El programa calcula los torques de referencia con RNEA, obtiene las aceleraciones mediante ABA y compara los resultados.

No solicita valores por teclado: utiliza `q`, `dq` y `ddq` definidos en `src/robot_control/robot_control/test_forward_dynamics.py`. Para repetir la prueba verificada:

```python
q = np.deg2rad([0, -50, 80, 0, -30, 0])
dq = np.array([0.1, 0.2, 0.1, 0.1, 0.1, 0.1], dtype=float)
ddq = np.array([0.2, 0.1, 0.2, 0.1, 0.1, 0.1], dtype=float)
```

Las pruebas de dinámica realizan cálculos numéricos; **no generan movimiento en Gazebo**.

### 10. Finalizar las pruebas

En la terminal de cada nodo o launch, pulsar **`Ctrl + C`**. Para cambiar de prueba, iniciar únicamente el launch y el nodo indicados en su sección.
