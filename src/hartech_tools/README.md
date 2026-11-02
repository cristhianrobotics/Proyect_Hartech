# HARTECH: herramienta paralela independiente — colores CAD, v0.1.1

[No verificado] La ejecución en ROS 2, RViz y Gazebo queda pendiente. Se entrega código con comprobaciones estáticas y numéricas; no se presenta como una simulación ejecutada o un agarre validado. Fecha: 14 de octubre de 2026.

Este paquete se añade junto a `robot_description` y `robot_control`. No reemplaza ninguno de sus archivos. Usa el robot del ZIP y las 54 ocurrencias de piezas del gripper HARTECH V2.1 de ayer, exportadas del ensamblaje nativo mediante Inventor sin guardar cambios. Las mallas están en metros. Los visuales usan COLLADA DAE con los colores RGB recuperados por cara desde Inventor, incluidos la carcasa negra y el grabado HARTECH claro. Se conservan los triángulos de las mallas anteriores. La iluminación, los reflejos y el sombreado de cada visor pueden diferir de Inventor.

## Actualización desde la entrega anterior

Consulta `../../LEEME_PRIMERO.md` del ZIP y `docs/ACTUALIZACION_COLORES.md`. Copia este paquete sobre el `hartech_tools` existente y recompila. No dupliques el paquete dentro de `src`. Ambos modelos, RViz y Gazebo Classic, incluyen el mismo Xacro de gripper actualizado. Los nombres de joints, controladores y comandos no cambian. Los STL se conservan como referencia; los visuales usan DAE con materiales internos.

## Instalación

Destino de esta entrega: Ubuntu 22.04, ROS 2 Humble y Gazebo Classic 11, que es la plataforma declarada por tu proyecto. No usar estos lanzadores con Gazebo Harmonic / `gz_ros2_control` sin migrarlos.

1. Descomprime la entrega en una carpeta temporal.
2. Copia **únicamente** la carpeta `src/hartech_tools` a `~/Hartech_ws/src/hartech_tools` (o a `src` de tu workspace real). No copies otra vez tus paquetes originales ni sobrescribas carpetas existentes.
3. En Ubuntu:

```bash
cd ~/Hartech_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/hartech_tools --ignore-src -r -y
colcon build --packages-up-to hartech_tools
source install/setup.bash
```

El paquete requiere que `robot_description` sea la versión entregada en tu ZIP. `robot_control` se conserva para tus ejercicios, pero no es necesario para abrir el gripper. En cada terminal carga `source /opt/ros/humble/setup.bash` y `source ~/Hartech_ws/install/setup.bash`.

## Ver el robot con el gripper en RViz

```bash
ros2 launch hartech_tools display.launch.py
```

La postura inicial del brazo es HOME. El deslizador `hartech_left_finger_joint` va de 0 a 0.025 m: representa el desplazamiento de UN dedo. La apertura total es el doble; 0.015 m equivale a 30 mm entre almohadillas. El otro dedo y el rotor siguen mediante `mimic`.

Alternativas:

```bash
ros2 launch hartech_tools display.launch.py pose:=zero
ros2 launch hartech_tools display.launch.py tool:=none
```

Ejecuta cada modalidad por separado. Cierra el launch anterior antes de abrir otro para evitar duplicar publicadores de estados y TF.

## Simular el conjunto en Gazebo Classic

```bash
ros2 launch hartech_tools gazebo.launch.py
```

Alternativa sin RViz: añade `rviz:=false`. Para arrancar sin herramienta: `tool:=none`. Los argumentos `pose:=home`, `pose:=zero`, `gui:=false` y `world:=/ruta/escena.world` también están disponibles.

El launch crea una sola entidad y un solo `robot_state_publisher`. El único plugin `gazebo_ros2_control` crea el gestor de controladores, con dos componentes de hardware simulados: brazo y gripper. Arranque: entidad → estados articulares → controlador del brazo → controlador del gripper. Si una etapa falla, se cierra el lanzamiento y se informa del error.

En otra terminal, comprueba:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

Deben aparecer activos `joint_state_broadcaster`, `arm_controller` y `gripper_controller` cuando se selecciona `parallel`. La lista de estados debe incluir los seis joints del brazo, los dos dedos y el rotor.

Abre y cierra con el nodo nuevo, esperando a que termine cada orden:

```bash
ros2 run hartech_tools set_opening.py --mm 50
ros2 run hartech_tools set_opening.py --mm 30
ros2 run hartech_tools set_opening.py --mm 0
```

`--mm` indica separación total entre las caras internas de las almohadillas. Se rechazan valores fuera de 0–50, NaN e infinitos. `--duration` es opcional, por defecto 3 s y mínimo 2 s. Son valores elegidos para simulación; no identifican el MG996R físico. El controlador usa la acción `/gripper_controller/follow_joint_trajectory` y recibe un único joint, `hartech_left_finger_joint`.

El brazo conserva `/arm_controller/follow_joint_trajectory` y los nombres `joint_1`…`joint_6`. Puedes enviar tus trayectorias articulares habituales, por ejemplo:

```bash
ros2 run robot_description send_trajectory.py --pose home --duration 5
```

No ejecutes los nodos que publican estados ficticios para RViz junto con Gazebo. Los nodos `test_forward_gazebo` y `test_inverse_gazebo` controlan el brazo; utiliza uno a la vez. No inicies además el launch antiguo.

## Qué contiene cada archivo

| Archivo | Función |
|---|---|
| `urdf/parallel_gripper.xacro` | Cuerpo, dedos, rotor, masa, inercia, colisiones simplificadas y TCP |
| `urdf/robot_with_tool.urdf.xacro` | Incluye tu robot original y añade la herramienta para RViz |
| `urdf/robot_with_tool.gazebo.xacro` | Incluye tu modelo original de Gazebo y añade el gripper simulado |
| `meshes/` | Geometría CAD actual de las 54 ocurrencias, agrupada mediante links |
| `config/controllers.yaml` | Conserva el controlador de seis ejes y añade el del gripper y sus estados |
| `launch/display.launch.py` | Visualización con sliders |
| `launch/gazebo.launch.py` | Lanzamiento separado del conjunto y sus controladores |
| `scripts/set_opening.py` | Orden de apertura en milímetros mediante acción ROS 2 |
| `docs/cad_export.json` | Procedencia, transformaciones y propiedades por grupo |
| `docs/original_zip_audit.json` | Inventario y comprobaciones del ZIP original |
| `docs/addon_validation.json` | Resultados de las comprobaciones de esta ampliación |

Los archivos `.xacro` generan un URDF completo; no hace falta mantener una copia manual adicional. Para exportar uno desde ROS 2:

```bash
xacro "$(ros2 pkg prefix --share hartech_tools)/urdf/robot_with_tool.urdf.xacro" > /tmp/hartech_with_gripper.urdf
```

## Brida, TCP y tus algoritmos

La conexión es `link_6 → tool0 → hartech_gripper_base → hartech_tcp`. `tool0` permanece en la cara de montaje de la brida. El TCP nuevo está a 100 mm en +Z de `tool0`, con la misma orientación de ese marco. El cambio de ejes CAD X → ROS Z se aplica en el montaje de la herramienta.

La cinemática DH actual coincide con `tool0`. Para obtener la pose del TCP:

```text
T_base_tcp(q) = T_base_tool0(q) · TransZ(0.100)
```

También puedes consultar TF:

```bash
ros2 run tf2_ros tf2_echo base_link hartech_tcp
```

Agregar el gripper al URDF NO cambia automáticamente tus funciones DH ni el objetivo que resuelve tu IK. Tus nodos existentes seguirán posicionando la brida. Para posicionar el centro de agarre, una etapa posterior debe usar esta transformación dentro de la FK y recalcular el Jacobiano del TCP. Si se conoce una pose objetivo completa del TCP: `T_base_tool0_deseada = T_base_tcp_deseada · inversa(T_tool0_tcp)`. Con solo XYZ y sin orientación elegida no basta con restar 100 mm al Z global.

Los ejercicios RNEA y ABA existentes leen expresamente `robot_6dof.urdf`, sin gripper. Siguen siendo ejercicios del brazo solo: añadir este paquete no les incorpora la masa de la herramienta. Tampoco se debe sustituirles directamente el URDF por uno con joints adicionales: ambos scripts presuponen seis grados de libertad. Para dinámica de brazo más herramienta se necesitará un modelo reducido con apertura fijada, o tratar correctamente los grados de libertad y restricciones del gripper. Esa extensión no se ha realizado ni se modificó tu código.

## Alcance físico de esta simulación

Masa CAD exportada: aproximadamente 0.265983 kg; no es una masa medida físicamente. El modelo conserva los cuerpos rígidos y el movimiento de dedos y rotor; no simula el contacto entre dientes de cremallera y piñón ni la electrónica del servo. El rotor reproduce la relación CAD de −100 rad/m respecto al joint maestro: apertura total 50 mm → rotor −2.5 rad.

Las colisiones son cajas separadas para carcasa/adaptador y para dedos, carros y almohadillas. No se coloca una caja que rellene la abertura entre dedos. Los contactos, rozamiento, amortiguamiento, velocidades y esfuerzos de simulación requieren validación. `selfCollide=false` sigue el planteamiento de tu simulación original; esta entrega no acredita ausencia de autocolisiones. El control ideal de posición y el movimiento visual de cierre no demuestran una fuerza de agarre ni que un objeto quede sujeto. No incluye ventosa, soldadura, acoplamiento automático ni un plugin de fijación de objetos.

## Añadir las siguientes herramientas

Elige la herramienta al arrancar: hoy solo están implementadas `parallel` y `none`. Para una ventosa o herramienta de soldadura se añade otra descripción y, si corresponde, otro controlador o plugin. Se reutiliza el brazo y se define el montaje, masa, colisiones y TCP de esa herramienta. Un cambio en caliente requeriría una implementación adicional; el argumento del launch no es un cambiador mecánico automático.

## Fuentes técnicas

Consultadas el 14 de octubre de 2026. Para interfaces de software se utilizan documentos oficiales, no una clasificación Q1/Q2 de revistas:

- [Universal Robots: composición de una celda mediante Xacro](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_tutorials/my_robot_cell/doc/assemble_urdf.html).
- [Universal Robots: marcos y conexión de herramientas a flange](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_description/doc/robot_frames.html). En UR se recomienda `flange`; tu proyecto llama `tool0` al marco de la cara de brida y por eso esta entrega se conecta allí.
- [ROS 2 Control Humble: Gazebo Classic y joints mimic](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html).
- [Controlador de trayectorias: parámetros Humble](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html).

Gazebo Classic terminó su mantenimiento en enero de 2025, según la documentación oficial. Se conserva aquí por compatibilidad con tu proyecto; migrar a Gazebo moderno es otro trabajo.
