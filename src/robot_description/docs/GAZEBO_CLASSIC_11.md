# Simulación del brazo en Gazebo Classic 11

Configuración destinada a **Ubuntu 22.04, ROS 2 Humble y Gazebo Classic 11**. El paquete conserva el nombre `robot_description` y el tipo `ament_cmake`.

## Qué se añadió

| Archivo | Función |
|---|---|
| `launch/gazebo.launch.py` | Abre Gazebo, publica el modelo, inserta el robot y activa los controladores en orden. RViz es opcional. |
| `urdf/robot_6dof.gazebo.xacro` | Modelo para Gazebo con base anclada al mundo, seis articulaciones controladas y plugin de simulación. |
| `urdf/gazebo_macros.xacro` | Macros exclusivas de simulación que reutilizan los parámetros físicos originales. |
| `config/gazebo_controllers.yaml` | Configuración de `joint_state_broadcaster` y `arm_controller`. |
| `config/gazebo_simulation.yaml` | Límites provisionales de simulación, amortiguamiento y cajas de colisión. |
| `worlds/robot_lab.world` | Mundo local con gravedad, suelo e iluminación, sin modelos externos. |
| `scripts/send_trajectory.py` | Cliente que envía objetivos de seis articulaciones y comprueba el resultado. |
| `docs/gazebo_validation.json` | Resultado de las verificaciones realizadas antes de entregar el paquete. |

Se extendieron `CMakeLists.txt`, `package.xml`, `README.md` y el manifiesto de entrega. No se convirtió el paquete a `ament_python`.

El CMake instala `worlds/` junto con los recursos anteriores. El cliente de trayectorias se instala mediante `install(PROGRAMS ...)` en `lib/robot_description` para ejecutarlo con `ros2 run`.

## Modelo conservado y aproximaciones

Los URDF/Xacro anteriores, `cad_macros.xacro`, `display.launch.py`, `robot_parameters.yaml`, RViz y los 14 meshes se conservan sin cambios. Se mantienen los nombres de las seis articulaciones, sus ejes, posiciones, orientaciones, límites angulares, masas, centros de masa e inercias.

El modelo nuevo para Gazebo añade únicamente:

- Un frame `world` y una unión fija `world_to_base`. El brazo completo no se marca como estático, por lo que sus articulaciones pueden moverse.
- Interfaces de comando `position` y de estado `position`, `velocity` y `effort`, mediante `gazebo_ros2_control/GazeboSystem` y `libgazebo_ros2_control.so`.
- Esfuerzo y velocidad positivos de ejemplo, exclusivamente en el modelo de simulación. Los esfuerzos son `[50, 50, 30, 10, 10, 5]` N·m y la velocidad límite es `0.8` rad/s por articulación. **Son valores provisionales; no proceden de fichas de motores ni de ensayos.** El YAML original mantiene sus ceros.
- Amortiguamiento de ejemplo de `0.1` por articulación.
- Una caja de colisión por cuerpo físico, calculada a partir de los límites del STL original en su frame local. Esto reduce el coste de contacto de las mallas CAD originales de más de dos millones de triángulos. Las mallas visuales y sus colores son los mismos.

Las cajas son aproximaciones envolventes, no geometría de contacto exacta. Se desactiva la autocolisión del modelo para evitar contactos artificiales entre cajas vecinas. No utilizar esta configuración para certificar distancias, colisiones o par requerido del robot físico.

`arm_controller` es un controlador de trayectorias con interfaz de posición. Representa un actuador ideal de posición; no modela motores, reductoras, saturación real ni un PID de par. Aunque se conservan las masas e inercias, esta integración no valida la dinámica de los actuadores reales. Un estudio de par requiere otra interfaz y parámetros identificados.

## 1. Instalar las dependencias

En el equipo con ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2controlcli
```

Desde el workspace, `rosdep` permite comprobar el resto de dependencias declaradas:

```bash
cd ~/Hartech_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 2. Copiar y construir

El ZIP completo contiene una única carpeta raíz `robot_description/`. Su `CMakeLists.txt` debe quedar en `~/Hartech_ws/src/robot_description/CMakeLists.txt`, y el launch nuevo en `~/Hartech_ws/src/robot_description/launch/gazebo.launch.py`. Evitar una carpeta `robot_description/robot_description/` adicional.

El ZIP de cambios contiene únicamente archivos nuevos o modificados y se combina con el paquete completo que ya se entregó. No basta por sí solo porque no contiene las mallas ni todos los recursos originales.

```bash
cd ~/Hartech_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_description
source install/setup.bash
```

Para verificar que los launch se instalaron:

```bash
ls "$(ros2 pkg prefix --share robot_description)/launch/"
```

Deben aparecer `display.launch.py` y `gazebo.launch.py`.

## 3. Abrir la simulación

```bash
ros2 launch robot_description gazebo.launch.py
```

Inicia en la pose HOME definida en el CAD. El plugin configura los valores iniciales, y `joint_state_broadcaster` publica los estados de Gazebo. Los argumentos disponibles son:

```bash
# Gazebo sin RViz
ros2 launch robot_description gazebo.launch.py rviz:=false

# Inicio en cero articular
ros2 launch robot_description gazebo.launch.py pose:=zero

# Simulación sin ventanas
ros2 launch robot_description gazebo.launch.py gui:=false rviz:=false
```

Se puede proporcionar otro mundo con `world:=/ruta/archivo.world`.

No ejecutar simultáneamente `display.launch.py`: ese launch utiliza un publicador GUI de estados articulares, mientras que en simulación los estados deben proceder de Gazebo.

## 4. Verificar los controladores

En otra terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Hartech_ws/install/setup.bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Los estados esperados son:

| Controlador | Tipo | Estado esperado |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | `active` |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `active` |

El plugin crea el único `controller_manager`; el launch no crea otro `ros2_control_node`. Después de insertar correctamente el robot se activa el broadcaster y, al terminar correctamente ese paso, el controlador del brazo. Si alguno falla, el launch termina con un mensaje de error.

## 5. Mover el robot

Con Gazebo ejecutándose y sin pausar:

```bash
# Prueba: J1 y J6 a +20 grados, J4 a +15 grados, manteniendo los demás en HOME.
ros2 run robot_description send_trajectory.py --pose test --duration 5

# Volver a HOME
ros2 run robot_description send_trajectory.py --pose home --duration 5

# Ir al cero nominal
ros2 run robot_description send_trajectory.py --pose zero --duration 5
```

También acepta seis ángulos en radianes:

```bash
ros2 run robot_description send_trajectory.py \
  --positions "0.2,-0.8,1.2,0.1,-0.4,0.2" --duration 5
```

El cliente espera la acción del controlador y estados de las seis articulaciones, rechaza valores no finitos o fuera de los límites CAD y comprueba que la duración permita una interpolación cúbica dentro de la velocidad provisional. No comprueba colisiones. Esperar a que termine una orden antes de ejecutar otra.

Las interfaces disponibles son:

| Interfaz | Nombre |
|---|---|
| Acción de trayectorias | `/arm_controller/follow_joint_trajectory` |
| Tópico de trayectorias | `/arm_controller/joint_trajectory` |
| Estados articulares | `/joint_states` |
| Transformaciones | `/tf`, `/tf_static` |
| Reloj de simulación | `/clock` |

## Alcance de las pruebas

Se comprobaron la expansión Xacro en HOME y cero, estructura URDF, conservación del modelo físico, rutas, configuración de controladores, colisiones envolventes, sintaxis y validaciones del cliente. **En el entorno de preparación no están instalados ROS 2, Gazebo, colcon ni los plugins, por lo que no se ejecutó una simulación real ni una construcción con colcon.** Activación de plugins/controladores, movimiento y aspecto en Gazebo quedan pendientes de comprobar en el equipo del usuario.

## Referencias oficiales utilizadas

- Gazebo Classic con ros2_control: https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html
- Controlador de trayectorias: https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
- Gestión y activación de controladores: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html
