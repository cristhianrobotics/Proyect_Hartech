# ROBOT 6DOF — Xacro y URDF

Descripción del robot del proyecto `GPT6`, con siete cuerpos rígidos y seis articulaciones rotacionales. El archivo principal editable es **`urdf/robot_6dof.urdf.xacro`**; **`urdf/robot_6dof.urdf`** es su expansión mediante Xacro. Ambos comparten las mallas y describen el mismo robot.

## Simulación en Gazebo Classic 11

Este paquete incluye ahora un launch específico para **ROS 2 Humble, Ubuntu 22.04 y Gazebo Classic 11**:

```bash
ros2 launch robot_description gazebo.launch.py
```

Añade `gazebo_ros2_control`, estados articulares desde Gazebo y un controlador de trayectorias para `joint_1` a `joint_6`. El modelo CAD original y su visualización en RViz se conservan. La configuración de simulación usa límites provisionales y colisiones simplificadas independientes del modelo original. La instalación, los archivos añadidos, los comandos de movimiento y las pruebas pendientes están en **`docs/GAZEBO_CLASSIC_11.md`**.

## Archivos

```text
robot_description/
├── urdf/
│   ├── robot_6dof.urdf.xacro   # Principal editable
│   ├── cad_macros.xacro        # Macros de eslabón y articulación
│   └── robot_6dof.urdf         # Resultado expandido
├── config/robot_parameters.yaml # Orígenes, inercias y límites
├── meshes/                    # DAE con colores CAD y STL de colisión, en metros
├── launch/display.launch.py   # robot_state_publisher, controles GUI y RViz
├── rviz/robot_6dof.rviz
├── docs/                      # Trazabilidad, informes y fuentes utilizadas
├── scripts/                   # Conversión y comprobaciones reproducibles
├── package.xml
├── CMakeLists.txt
└── LICENSE
```

**Conservar el paquete completo.** Copiar únicamente los dos archivos principales deja fuera las mallas y las dependencias del Xacro.

## Alcance y datos pendientes

- Las seis articulaciones tienen **`axis xyz="0 0 1"`**. Sus marcos se orientaron para que Z local coincida con el eje físico; no se cambiaron los ejes del robot respecto a su geometría.
- Unidades: metros, radianes, kg y kg·m². Las mallas ya están en metros: **no aplicar otra escala de 0,001**.
- Masa total de los siete eslabones, tomada de la exportación CAD corregida: **8,008416688468413 kg**. Es una propiedad del modelo CAD, no una medición del robot construido.
- **Velocidad y esfuerzo = 0**, por indicación del usuario, porque todavía no se proporcionaron límites físicos. Permite describir y visualizar la cinemática; no constituye una configuración para controlar el robot ni ejecutar trayectorias.
- Los límites angulares se conservaron del proyecto y son **provisionales**. La documentación original no acredita ausencia de colisiones para todas las combinaciones, montajes actuales o comportamiento del cableado. Véase `docs/sources/LIMITES_ARTICULARES.md`.
- Las inercias son las corregidas de la exportación V2, transformadas al nuevo marco de cada link. No incluyen una herramienta añadida, una carga útil ni inercias reflejadas de los motores.
- `collision` utiliza la misma superficie CAD triangulada que `visual`. No son mallas convexas ni simplificadas para simulación dinámica. No se ejecutó una nueva validación de colisiones.
- **Limitación verificada de las mallas:** los conjuntos de sólidos no forman una única superficie cerrada manifold. Las mallas de `base_body`, `link_1` y `link_2` contienen bordes abiertos en la triangulación de los motores comerciales. Se conservaron las superficies del CAD, incluyendo ocho caras cónicas por motor convertidas a NURBS para triangularlas. No se rellenaron huecos con geometría inventada. Antes de usarlas para dinámica o validación de colisiones se requiere preparar y verificar mallas de colisión adecuadas. Los recuentos están en `docs/validation.json`.
- Los subconjuntos son cuerpos rígidos: no se simulan transmisiones, poleas, rotores internos, encoders o cables como articulaciones adicionales.
- Los materiales de las mallas visuales DAE conservan los colores exportados en el STEP, por pieza y por superficie. Xacro y URDF utilizan esas mismas mallas. Las mallas STL de colisión, geometría, marcos, articulaciones, masas e inercias se mantienen. El sombreado final depende de la iluminación del visor; no se reproducen texturas ni acabados exclusivos de Inventor. La trazabilidad de colores y la comparación exacta de triángulos están en `docs/color_audit.json`.
- **No se ejecutaron ROS 2, RViz ni Gazebo en este equipo.** Las comprobaciones realizadas son de expansión, estructura, cinemática, geometría e inercias; el informe está en `docs/validation.json`.

## Eslabones y cero articular

| Cuerpo CAD | Link ROS | Articulación de entrada | Límites CAD |
|---|---|---|---|
| BASE_SUBASSEMBLY | base_body | fija a base_link | — |
| HOMBRO_SUBASSEMBLY | link_1 | joint_1 | −160° a +160° |
| BRAZO_SUBASSEMBLY | link_2 | joint_2 | −95° a +5° |
| ANTEBRAZO_SUBASSEMBLY | link_3 | joint_3 | −110° a +110° |
| MUNECA_J4_SUBASSEMBLY | link_4 | joint_4 | −150° a +150° |
| MUNECA_J5_SUBASSEMBLY | link_5 | joint_5 | −70° a +70° |
| BRIDA_J6_SUBASSEMBLY | link_6 | joint_6 | −160° a +160° |

`base_link` es un marco raíz sin masa y coincide con el origen CAD. `base_body` contiene la geometría e inercia de la base. Este montaje mantiene la inercia física fuera del link raíz. `tool0` está en el centro de la cara nominal de montaje de la brida, a 9 mm en Z local de `link_6`; **no es un TCP calibrado de una herramienta**.

**Todos los ángulos en cero corresponden a la pose horizontal nominal**. La cadena tiene J1 a 178 mm y J2 a 273 mm de altura; separaciones J2–J3 = 160 mm, J3–J4 = 130 mm, J4–J5 = 45 mm y J5–J6 = 104 mm. El centro de `tool0` en cero queda en (448, 0, 273) mm.

La pose guardada **HOME** es `[0, −50, 80, 0, −30, 0]` grados. El lanzador inicia con esos valores. Cambiar `pose:=zero` muestra el cero nominal.

## Abrir en ROS 2

Con ROS 2 y las dependencias instaladas, copiar la carpeta `robot_description` dentro de `~/Hartech_ws/src/`. Desde ese workspace:

```bash
cd ~/Hartech_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_description display.launch.py
```

Para abrir directamente el URDF expandido:

```bash
ros2 launch robot_description display.launch.py \
  model:="$(ros2 pkg prefix --share robot_description)/urdf/robot_6dof.urdf"
```

Para abrir la pose horizontal:

```bash
ros2 launch robot_description display.launch.py pose:=zero
```

Los deslizadores representan ángulos en radianes y permiten inspección manual. No envían órdenes al robot físico. Estas instrucciones siguen la estructura del paquete ROS 2; su ejecución local queda pendiente.

## Instalación de scripts auxiliares

Todos los scripts se instalan en `share/robot_description/scripts/` y se pueden ejecutar con `python3`. Son herramientas de exportación y verificación, no nodos del launch. Las dependencias CAD de `scripts/requirements-export.txt` son opcionales y no se necesitan para abrir el robot en RViz. Los scripts de reexportación requieren la carpeta externa `GPT6`, que no viene en estos ZIP; `render_preview.py` también requiere el módulo Python `vtk`.

El regenerador se instala además como ejecutable mediante `install(PROGRAMS ...)`; desde un entorno ROS 2 configurado se puede invocar con `ros2 run robot_description regenerate_urdf.py`. Resuelve los recursos tanto en el código fuente como en una instalación normal o con enlaces simbólicos. Solo modifica el URDF cuando el usuario lo ejecuta.

## Editar y regenerar

Editar `config/robot_parameters.yaml` para los valores físicos, o el Xacro para modificar la estructura. Después:

```bash
python3 scripts/regenerate_urdf.py
```

También se puede usar directamente Xacro:

```bash
xacro urdf/robot_6dof.urdf.xacro > urdf/robot_6dof.urdf
```

El parámetro `mesh_prefix` permite sustituir `package://robot_description/meshes` por una ruta absoluta si un visor externo no resuelve paquetes ROS. Por ejemplo, `mesh_prefix:=/ruta/robot_description/meshes`. El Xacro admite un `parameters_file` alternativo; una ruta absoluta evita ambigüedades fuera del paquete.

`build_description.py` es un reconstruidor desde los CSV originales y sobrescribe la configuración: **no usarlo para conservar ajustes manuales**. Para cambios habituales basta `regenerate_urdf.py`.

## Procedencia y verificación

La automatización COM de Inventor no estaba registrada en el equipo. Por ello las mallas proceden de **`GPT6/06_STEP/ROBOT_6DOF.step`**, exportado por Inventor el 20/09/2026, y no de una nueva conversión nativa del IAM. El registro `CAD_LOG.txt` identifica esa exportación y su tamaño de 42 329 112 bytes. Los **458 archivos nativos** del inventario, incluido `04_Ensamblajes/ROBOT_6DOF.iam`, coinciden por SHA-256 con el manifiesto conservado en el proyecto. Véase `docs/source_audit.json`.

El STEP conserva los nombres, las ocurrencias y las transformaciones de los siete subconjuntos. La conversión excluye geometría marcada como referencia en el BOM y conserva las ocurrencias físicas. Las transformaciones de HOME se contrastan con el URDF, y los seis ejes/orígenes se contrastan con `GEOMETRIA_FINAL_JOINTS.csv`. Los momentos principales de inercia se contrastan con los valores auditados de Inventor. Los datos de masa utilizados tienen export_id **2026-09-20T12:44:14.5367655Z**.

Para reproducir la exportación completa, mantener `GPT6` junto al paquete e instalar en un entorno Python separado las dependencias de `scripts/requirements-export.txt`. Ejecutar, en este orden:

```bash
python3 scripts/audit_sources.py
python3 scripts/inspect_step.py
python3 scripts/build_description.py
python3 scripts/export_meshes.py
python3 scripts/export_visual_colors.py
python3 scripts/validate_description.py
```

La comprobación de 100 configuraciones compara dos formulaciones matemáticas de la cadena. No sustituye ensayos físicos ni un nuevo barrido en Inventor. La comparación con el STEP y la exportación nativa de ejes se realiza en HOME.

Se exportaron **520 ocurrencias físicas** a siete mallas, con **2 027 197 triángulos** en total y tolerancia nominal de triangulación de 0,05 mm. La discrepancia máxima de volumen integrado entre una malla de ocurrencia y su CAD fue **1,34 %**, en el motor J2. Esta comprobación no convierte una superficie abierta en un sólido válido; las masas e inercias se obtuvieron del CSV CAD corregido, no de las mallas. `docs/mesh_audit.json` conserva el detalle por componente.

El paquete conserva el mantenedor, la licencia Apache-2.0, la versión y el tipo `ament_cmake` del paquete base creado por el usuario. Los informes originales de CAD se conservan como antecedentes; las verificaciones específicas de esta adaptación están en `docs/MIGRACION.md` y `docs/migration_validation.json`.

## Referencias técnicas oficiales

- [ROS 2: modelos articulados con URDF](https://docs.ros.org/en/humble/Tutorials/URDF/Building-a-Movable-Robot-Model-with-URDF.html).
- [ROS 2: documentación de Xacro](https://docs.ros.org/en/ros2_packages/jazzy/api/xacro/modules.html).
- [ROS: joint_state_publisher](https://docs.ros.org/en/ros2_packages/rolling/api/joint_state_publisher/index.html).
- [Autodesk: XYZMomentsOfInertia](https://help.autodesk.com/cloudhelp/2022/ENU/Inventor-API/files/MassProperties_XYZMomentsOfInertia.htm), también documentado en la corrección V2 del proyecto.
