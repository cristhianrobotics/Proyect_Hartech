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

---

## Ampliación de la guía — Gripper, cinemática avanzada y control dinámico (continuación)

### 11. HARTECH con gripper — Paquete `hartech_tools`

Se incorporó un gripper paralelo al modelo original, sin reemplazar los programas anteriores del brazo. La ampliación utiliza el paquete `hartech_tools` para la descripción Xacro/URDF, las mallas, los lanzadores de RViz/Gazebo y el control de apertura.

**Elementos principales:**

| Elemento | Función |
|---|---|
| `src/hartech_tools/urdf/parallel_gripper.xacro` | Modelo del gripper y sus articulaciones. |
| `src/hartech_tools/urdf/robot_with_tool.urdf.xacro` | HARTECH con herramienta para RViz. |
| `src/hartech_tools/urdf/robot_with_tool.gazebo.xacro` | HARTECH con gripper para Gazebo Classic. |
| `src/hartech_tools/launch/display.launch.py` | Visualización con sliders. |
| `src/hartech_tools/launch/gazebo.launch.py` | Simulación con controladores de posición del brazo y el gripper. |
| `src/hartech_tools/scripts/set_opening.py` | Apertura/cierre en milímetros. |

**RViz con gripper (Terminal 1):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch hartech_tools display.launch.py
```

Para iniciar con las articulaciones del brazo en cero:

```bash
ros2 launch hartech_tools display.launch.py pose:=zero
```

**Gazebo Classic con gripper (Terminal 1, después de cerrar RViz):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch hartech_tools gazebo.launch.py rviz:=false
```

**Terminal 2 — apertura del gripper** (ejecutar una orden por vez):

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run hartech_tools set_opening.py --mm 50
ros2 run hartech_tools set_opening.py --mm 30
ros2 run hartech_tools set_opening.py --mm 0
```

`--mm` indica la **apertura total entre almohadillas**, de 0 a 50 mm. En la configuración dinámica posterior se usa, de forma habitual, **30 mm de apertura fija**; eso no implica que la dinámica simule la acción de abrir o cerrar los dedos.

**Comprobación de los controladores de posición:**

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Para el Gazebo de posición con gripper se esperan `joint_state_broadcaster`, `arm_controller` y `gripper_controller` activos.

#### 11.1. Diferencia entre `tool0` y `hartech_tcp`

El brazo original termina en el marco `tool0`. Al añadir el gripper, se utiliza `hartech_tcp`, desplazado **100 mm en el eje Z local de `tool0`**, manteniendo su orientación:

```text
T_base_tcp(q) = T_base_tool0(q) · TransZ(0.100 m)
```

Por tanto, las posiciones XYZ de `tool0` **no son equivalentes** a las del TCP del gripper. Las funciones de cinemática de `robot_control_griper` trabajan con ese TCP; el brazo sigue teniendo seis articulaciones controladas.

### 12. Cinemática directa e inversa con gripper — RViz y Gazebo

Las implementaciones con gripper están en:

```text
~/Hartech_ws/src/robot_control/robot_control_griper/
```

Algunos módulos reutilizados son `fkine_gripper.py`, `ikine_gripper.py` y `modelo_dinamico_gripper.py`. Los nodos con gripper son adicionales a los originales de `robot_control/robot_control/`.

**RViz sin sliders (Terminal 1):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch hartech_tools display_control.launch.py
```

Si ese lanzador no está en la versión instalada de `hartech_tools`, utilizar el lanzador genérico con el modelo del gripper:

```bash
ros2 launch robot_description display_control.launch.py model:=$(ros2 pkg prefix --share hartech_tools)/urdf/robot_with_tool.urdf.xacro
```

**Terminal 2 — ejecutar uno por vez:**

```bash
ros2 run robot_control test_forward_kinematic_gripper
# o bien
ros2 run robot_control test_inverse_kinematics_gripper
```

**Gazebo con controlador de posición (Terminal 1):**

```bash
ros2 launch hartech_tools gazebo.launch.py rviz:=false
```

**Terminal 2 — ejecutar uno por vez:**

```bash
ros2 run robot_control test_forward_gripper_gazebo
# o bien
ros2 run robot_control test_inverse_gripper_gazebo
```

Estas pruebas envían movimientos de las **seis articulaciones del brazo**; la apertura de la pinza se gobierna por separado. Para el TCP del gripper, una posición de HOME de referencia es aproximadamente `[0.4674, 0, 0.3081]` m (con el modelo y HOME utilizados en las pruebas).

### 13. Jacobiano geométrico 6×6 con y sin gripper

Se implementaron los Jacobianos para `tool0` y para `hartech_tcp`, incluidas pruebas numéricas de validación.

**Una sola terminal (sin Gazebo):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_jacobian_6dof
ros2 run robot_control test_jacobian_gripper
```

Archivos principales:

```text
src/robot_control/robot_control/jacobian_6dof.py
src/robot_control/robot_control_griper/jacobian_gripper.py
```

El Jacobiano relaciona las velocidades articulares con el *twist* del efector: tres componentes lineales y tres angulares. Al desplazar el punto de interés de `tool0` al TCP, **cambia la parte lineal del Jacobiano**, aunque el robot conserve sus seis grados de libertad.

### 14. Análisis de singularidades

Se incorporaron pruebas de rango y valores singulares del Jacobiano 6×6, tanto con herramienta como sin ella.

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_singularities_6dof
ros2 run robot_control test_singularities_gripper
```

En la familia de configuraciones estudiada, al aproximar **J5 a 0°** aparece una singularidad de muñeca: se alinean los ejes de rotación de J4 y J6 y el rango puede bajar de 6 a 5. Esto explica la conveniencia de utilizar mínimos cuadrados amortiguados (DLS) en control diferencial cerca de configuraciones singulares. Las pruebas son numéricas y no comprueban colisiones.

### 15. Cinemática inversa de pose completa — XYZ + RPY (6D)

Además de la IK de posición XYZ, se desarrolló una IK que busca simultáneamente **posición y orientación**, con convención:

```text
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

**En RViz, sin gripper:**

```bash
# Terminal 1
ros2 launch robot_description display_control.launch.py
# Terminal 2
ros2 run robot_control test_inverse_pose_6d
```

**En RViz, con gripper (reiniciar el entorno anterior):**

```bash
# Terminal 1
ros2 launch hartech_tools display_control.launch.py
# Terminal 2
ros2 run robot_control test_inverse_pose_gripper
```

**En Gazebo, sin gripper:**

```bash
# Terminal 1
ros2 launch robot_description gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_inverse_pose_gazebo
```

**En Gazebo, con gripper:**

```bash
# Terminal 1
ros2 launch hartech_tools gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_inverse_pose_gripper_gazebo
```

Los nodos admiten las palabras `otra`, `home`, `actual` y `salir`, o seis valores `X Y Z roll pitch yaw` (metros y grados). Una postura de prueba se obtiene con `otra` y se puede regresar con `home`.

> **No confundir:** RViz muestra la solución cinemática; Gazebo ejecuta un movimiento con sus controladores. El resultado depende de los límites articulares, la configuración de partida y la posibilidad de alcanzar la pose solicitada.

### 16. Cinemática diferencial — Análisis numérico

Se comparó la pseudoinversa del Jacobiano con mínimos cuadrados amortiguados (DLS), especialmente al acercarse a una singularidad.

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_differential_6dof
ros2 run robot_control test_differential_gripper
```

Estos programas calculan `qdot` a partir de un *twist* deseado y generan resultados numéricos; **no ordenan movimientos en Gazebo**.

### 17. Control cinemático diferencial en Gazebo

También se implementó seguimiento por realimentación de posición cartesiana y de pose 6D usando el controlador de **posición articular** de Gazebo; estos nodos **no utilizan control por torques**.

**A. Regulación diferencial XYZ sin gripper:**

```bash
# Terminal 1
ros2 launch robot_description gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_diffkine_gazebo_hartech
```

**B. Regulación diferencial XYZ con gripper:**

```bash
# Terminal 1
ros2 launch hartech_tools gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_diffkine_gripper_gazebo
```

**C. Control diferencial de pose XYZ + orientación, sin gripper:**

```bash
# Terminal 1
ros2 launch robot_description gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_diffkine_pose_gazebo_hartech
```

**D. Control diferencial de pose XYZ + orientación, con gripper:**

```bash
# Terminal 1
ros2 launch hartech_tools gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_diffkine_pose_gripper_gazebo
```

Cerrar el Gazebo y el nodo anteriores antes de ejecutar otra modalidad. No deben existir dos nodos enviando simultáneamente trayectorias a `/arm_controller/follow_joint_trajectory`.

### 18. Trayectoria cartesiana cuadrada XY — Control cinemático (Laboratorio 6)

Se realizaron trayectorias cuadradas de **6 × 6 cm** en el plano XY. La versión con gripper V2 se diseñó para mantener el TCP **orientado verticalmente hacia abajo**, simulando conceptualmente el uso de un plumón (sin agregar un plumón físico).

**Sin gripper, con Gazebo de posición:**

```bash
# Terminal 1
ros2 launch robot_description gazebo.launch.py rviz:=false
# Terminal 2
ros2 run robot_control test_trayectoria_cartesiana_gazebo_hartech
```

**Con gripper — versión V2 validada del cuadrado:**

```bash
# Terminal 1
ros2 launch hartech_tools gazebo.launch.py rviz:=false
# Terminal 2 (desde ~/Hartech_ws)
cd ~/Hartech_ws
source install/setup.bash
python3 src/robot_control/robot_control_griper/test_trayectoria_cartesiana_gripper_gazebo_v2.py
```

La versión V2 realiza la secuencia **HOME → postura de escritura → cuadrado XY**. El código original fue `test_trayectoria_cartesiana_gripper_gazebo_v2.py`, por lo que **no debe confundirse** con el modo `--tipo cuadrado` del control dinámico. Sus registros se guardan en `~/Hartech_ws/resultados_lab6/`.

> **Estado:** el cuadrado mediante control cinemático con gripper ya se ejecutó satisfactoriamente. Se conserva como referencia válida de geometría y orientación. No acredita todavía un cuadrado controlado por torques.

### 19. Modelo dinámico de HARTECH con gripper — Pinocchio

Se amplió el modelo dinámico para incluir las masas e inercias del gripper. Para mantener el brazo con **6 GDL**, las articulaciones del gripper se fijan a una apertura determinada (30 mm por defecto) al construir el modelo reducido de Pinocchio.

**RNEA: dinámica inversa con gripper:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_dynamics_gripper
```

**ABA: dinámica directa con gripper:**

```bash
ros2 run robot_control test_forward_dynamics_gripper
```

**Otra apertura fija, únicamente para el cálculo dinámico:**

```bash
ros2 run robot_control test_dynamics_gripper --mm 50
```

El análisis proporciona `M(q)`, términos de Coriolis/centrífugos, `g(q)`, torques RNEA y aceleraciones ABA. **Son cálculos independientes de Gazebo**: no modelan cargas agarradas, contactos ni la apertura dinámica de la pinza durante el movimiento.

### 20. Control PD con compensación de gravedad — Simulación ideal ABA

Se implementó el control:

```text
τ = Kp (qd − q) + Kd (dqd − dq) + g(q)
```

La respuesta se evaluó en Pinocchio utilizando ABA y tres niveles de amortiguamiento: `sub`, `critico` y `sobre`.

**Ejecutar la comparación completa sin Gazebo:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_pd_gravedad_gripper --modo todos
```

**Visualizar una única prueba en RViz, sin sliders:**

```bash
# Terminal 1
ros2 launch hartech_tools display_control.launch.py
# Terminal 2
ros2 run robot_control test_pd_gravedad_gripper --modo critico --rviz
```

**Crear las gráficas de los tres modos:**

```bash
python3 src/robot_control/robot_control_griper/graficas_pd_gripper.py
```

Los CSV y PNG se guardan en `~/Hartech_ws/resultados_lab7_gripper/`. La simulación **ideal ABA** alcanzó la referencia en los ensayos realizados, pero no garantiza la misma precisión bajo la física de Gazebo.

### 21. Control PD + gravedad por torques en Gazebo (Laboratorio 7)

Para aplicar los esfuerzos reales del controlador sobre las articulaciones simuladas se agregó un lanzador específico:

```text
src/hartech_tools/launch/gazebo_esfuerzo.launch.py
src/hartech_tools/config/controllers_esfuerzo.yaml
src/robot_control/robot_control_griper/test_pd_gravedad_gripper_gazebo.py
```

A diferencia de `gazebo.launch.py`, este entorno usa `arm_effort_controller` y la interfaz `effort` de `joint_1` a `joint_6`.

**PD + gravedad con inicio automático (Terminal 1):**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch hartech_tools gazebo_esfuerzo.launch.py
```

Para verificar la interfaz y el controlador:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /arm_effort_controller/commands --once
```

El lanzador de esfuerzo utiliza un **mundo temporal con gravedad inicialmente cero** para evitar la caída del robot antes de activar el control. Luego la gravedad debe restablecerse bajo control. Los límites de torque usados en Gazebo son **provisionales de simulación**, no especificaciones medidas del hardware.

**Resultados observados:** se confirmó que los seis comandos de esfuerzo llegaban a Gazebo. Una corrección de filtrado de velocidad y ganancias redujo notablemente las vibraciones, pero quedó un **error articular residual del orden de 6–7°** en ensayos anteriores. Por tanto, el PD + gravedad en Gazebo debe figurar como **implementado, no plenamente validado en precisión**.

> **IMPORTANTE:** cuando `gazebo_esfuerzo.launch.py` arranca el PD automáticamente, **no** iniciar en paralelo otro nodo que publique a `/arm_effort_controller/commands`.

### 22. Comparación de PD+g, PID+g y dinámica inversa — ABA ideal

Se incorporaron `dinamica_torque_gripper.py`, `test_dinamica_inversa_gripper_aba.py` y `graficas_control_dinamico_gripper.py` para comparar tres métodos: `pdg` (PD+g), `pidg` (PD+integral+g) y `ctc` (*Computed Torque Control*).

**Una sola terminal, sin Gazebo:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_dinamica_inversa_gripper_aba
```

El programa genera, entre otros:

```text
~/Hartech_ws/resultados_lab7_gripper/comparacion_aba_pdg.csv
~/Hartech_ws/resultados_lab7_gripper/comparacion_aba_pidg.csv
~/Hartech_ws/resultados_lab7_gripper/comparacion_aba_ctc.csv
~/Hartech_ws/resultados_lab7_gripper/comparacion_aba_error_articular.png
~/Hartech_ws/resultados_lab7_gripper/comparacion_aba_error_tcp.png
```

**Resultado documentado:** en las gráficas comparativas de ABA ideal, el método `ctc` presentó los menores picos de error (aprox. **0.088° articular** y **0.53 mm en TCP**). Esto valida la comparación **en ese modelo ideal**, pero no sustituye pruebas de estabilidad en Gazebo.

### 23. Regulación dinámica por torques — PID+g y CTC en Gazebo

El nodo `test_control_dinamico_gripper_gazebo.py` incorpora varios métodos y tipos de referencia. Reutiliza `referencias_dinamicas_gripper.py` para generar posiciones, velocidades y aceleraciones deseadas.

**Terminal 1 — Gazebo de esfuerzo SIN iniciar el PD anterior:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 launch hartech_tools gazebo_esfuerzo.launch.py autostart_pd:=false
```

**Terminal 2 — ejecutar solo UNA modalidad por simulación:**

```bash
cd ~/Hartech_ws
source install/setup.bash

# PID con compensación de gravedad: ensayo del error residual
ros2 run robot_control test_control_dinamico_gripper_gazebo --metodo pidg --tipo regulacion

# O BIEN: control por dinámica inversa (CTC), en otro arranque desde HOME
ros2 run robot_control test_control_dinamico_gripper_gazebo --metodo ctc --tipo regulacion
```

La referencia nominal de regulación fue:

```text
q_objetivo [grados] = [20, -55, 75, 10, -25, 10]
```

La dinámica inversa utiliza RNEA y puede expresarse como:

```text
τ = M(q)[ddqd + Kd(dqd − dq) + Kp(qd − q)] + C(q,dq)dq + g(q)
```

Se implementó una transición progresiva de gravedad **0g → 1g** para reducir movimientos bruscos durante el arranque. En la prueba de regulación CTC se observó un descenso importante del error articular, pero también un error del TCP persistente (del orden de **18 mm** en un ensayo), por lo que **la precisión conjunta XYZ/articular no quedó completamente validada**.

> `autostart_pd:=false` es obligatorio en estas pruebas si se utiliza este segundo nodo: evita tener dos publicadores de torques activos a la vez. Reiniciar Gazebo desde HOME entre modalidades.

### 24. Seguimiento de trayectoria articular mediante control dinámico (CTC)

Se generaron referencias articulares de posición, velocidad y aceleración usando interpolación quíntica. El recorrido previsto es:

```text
HOME → otra configuración → HOME
```

**Terminal 1 — Gazebo de esfuerzo, desde HOME:**

```bash
ros2 launch hartech_tools gazebo_esfuerzo.launch.py autostart_pd:=false
```

**Terminal 2 — seguimiento dinámico:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 run robot_control test_control_dinamico_gripper_gazebo --metodo ctc --tipo trayectoria
```

Esta modalidad fue ejecutada durante el desarrollo. Al documentar su validación deben comprobarse el seguimiento articular, los límites, los torques y las gráficas del CSV. **No supone que el TCP recorra una línea recta cartesiana.**

### 25. Cuadrado XY de 6×6 cm mediante torques — Prueba pendiente de corregir

**Objetivo:** reutilizar la trayectoria XY ya validada en el Laboratorio 6, pero aplicar **torques dinámicos** sobre el brazo con gripper, en vez de órdenes de posición articular.

El generador se encuentra en:

```text
src/robot_control/robot_control_griper/referencias_dinamicas_gripper.py
```

El modo de ejecución se selecciona en:

```text
src/robot_control/robot_control_griper/test_control_dinamico_gripper_gazebo.py
```

El comando previsto para la prueba es:

```bash
# Solo con el simulador de esfuerzo preparado SIN otro publicador de torque
ros2 run robot_control test_control_dinamico_gripper_gazebo --metodo ctc --tipo cuadrado
```

**Estado real de las pruebas:** la referencia deseada representa el cuadrado, pero la trayectoria ejecutada no lo reprodujo correctamente. Se observaron desviaciones grandes, estancamientos del avance adaptativo y, en algunas versiones, activación de protecciones durante HOME o la preparación para la postura de escritura. Las gráficas incluyeron errores de decenas de grados y centímetros para intentos fallidos.

Se ensayaron modificaciones V3, V4 y V5 del arranque/seguimiento y una variante con corrección de torque; **ninguna de estas pruebas acreditó el cuadrado dinámico como completado**. Se mantiene como referencia exitosa únicamente el cuadrado **cinemático** del apartado 18. La variante experimental de controlador robusto también falló durante el arranque y no debe presentarse como solución validada.

> **No ejecutar este modo como demostración aprobada.** Antes de reintentarlo, revisar los registros `q`, `qd`, `dq`, `tau`, `g(q)`, velocidades y las diferencias entre el modelo Pinocchio y la física de Gazebo. Mantener activas las protecciones y detener la simulación si aparecen movimientos inesperados. El nodo no sustituye una parada segura del simulador físico.

### 26. Gráficas y archivos de resultados — Laboratorios 6 y 7

**Laboratorio 6, cuadrado cinemático con gripper:**

```text
~/Hartech_ws/resultados_lab6/
```

**Laboratorio 7, simulaciones dinámicas y ensayos de torques:**

```text
~/Hartech_ws/resultados_lab7_gripper/
```

Para generar comparaciones a partir de los CSV de Gazebo:

```bash
cd ~/Hartech_ws
source install/setup.bash
python3 src/robot_control/robot_control_griper/graficas_control_dinamico_gripper.py
```

Para analizar **un CSV determinado** en vez de los registros más recientes:

```bash
python3 src/robot_control/robot_control_griper/graficas_control_dinamico_gripper.py \
  resultados_lab7_gripper/torque_ctc_cuadrado_ARCHIVO.csv
```

Sustituir `torque_ctc_cuadrado_ARCHIVO.csv` por el nombre real del registro. El script puede generar:

```text
comparacion_gazebo_control_dinamico.png
cuadrado_dinamico_gripper_xy.png
```

En `cuadrado_dinamico_gripper_xy.png` la trayectoria puede incluir **el acercamiento desde HOME**; no interpretar toda la curva de preparación como si fueran lados del cuadrado. Un CSV de una ejecución interrumpida representa un **intento fallido** y no acredita la precisión del seguimiento.

### 27. Estado consolidado y comprobaciones útiles

| Desarrollo | Estado documentado |
|---|---|
| Modelo original del brazo, RViz y Gazebo | Realizado. |
| Integración del gripper, apertura y TCP | Realizado. |
| Cinemática directa e inversa XYZ con y sin gripper | Realizado. |
| IK de pose 6D, Jacobiano y singularidades | Realizado. |
| Control cinemático diferencial XYZ y 6D | Realizado. |
| Cuadrado cartesiano con gripper mediante control cinemático | **Ejecutado satisfactoriamente.** |
| Dinámica RNEA/ABA del brazo y del gripper | Realizado. |
| Control PD+gravedad y comparación de amortiguamientos en ABA | **Validado en simulación ideal.** |
| Comparación PD+g / PID+g / CTC en ABA ideal | **Validada en simulación ideal.** |
| PD+gravedad por esfuerzo en Gazebo | Aplicado; precisión residual pendiente. |
| CTC de regulación en Gazebo | Ejecutado; precisión cartesiana pendiente de verificar. |
| Seguimiento dinámico HOME→otra→HOME | Ensayado; revisar registros para validación cuantitativa. |
| **Cuadrado mediante torques en Gazebo** | **No validado: pendiente de corrección.** |

**Comprobar ejecutables instalados en ROS 2:**

```bash
cd ~/Hartech_ws
source install/setup.bash
ros2 pkg executables robot_control
```

Si algún `ros2 run robot_control <nodo>` no aparece, comprobar que el archivo `.py` está en el paquete correspondiente, que su entrada `console_scripts` se añadió a `src/robot_control/setup.py` y recompilar:

```bash
colcon build --packages-select robot_control
source install/setup.bash
```

Los scripts auxiliares (por ejemplo, generadores de referencias y gráficas) no necesitan una entrada `console_scripts` si son **importados** por otro nodo o ejecutados directamente con `python3`.

### 28. Guardar la documentación actualizada en GitHub


```bash
cd ~/Hartech_ws
git status
git add README.md
git commit -m "docs: ampliar guia HARTECH gripper cinematica y control dinamico"
git push origin main
```
