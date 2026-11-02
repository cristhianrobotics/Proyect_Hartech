# Colores CAD — hartech_tools 0.1.1

14 de octubre de 2026. Alcance: solo apariencia visual.

Los visuales usan COLLADA DAE con materiales por cara, obtenidos de `generic_diffuse` de los assets nativos de Inventor. Se respetan las apariencias de cara y las sobrescrituras de ocurrencia. Las letras blancas corresponden a las caras del grabado existente; no se añade una textura ni geometría nueva.

| Zona | RGB del CAD |
|---|---|
| Carcasa | 22, 28, 37 |
| Grabado HARTECH | 231, 236, 242 |
| Dedos | 193, 203, 215 |
| Carros y cremalleras | 130, 144, 160 |
| Piñón | 221, 164, 34 |
| Almohadillas | 31, 34, 38 |
| Adaptador y chasis | 42, 50, 63 |

Se exportaron 17 colores, incluyendo servo, guías y tornillería. Los materiales almacenan esos RGB divididos entre 255. La iluminación y el acabado de reflejos de Inventor no se reproducen de forma idéntica entre motores gráficos; los parámetros Phong de reflejo son parámetros de visualización.

## Pruebas

- Lectura independiente de los 54 DAE con PyCollada y comparación de cada triángulo y color.
- Triángulos DAE idénticos a los float32 de los STL de la entrega anterior; los STL también se conservan byte por byte.
- Sin texturas ni archivos externos necesarios para los materiales.
- Eliminados los colores generales URDF de cada visual para utilizar los materiales internos de la malla.
- Articulaciones, límites, inercias, colisiones, montaje y TCP idénticos.
- Lanzadores, controladores, nodo de apertura y reglas de instalación idénticos.
- Archivos CAD sin cambios, comprobados por hashes antes y después.
- Expansión de las ocho variantes: RViz/Gazebo, parallel/none, home/zero.

El informe por pieza está en `color_validation.json`. No se ha ejecutado esta nueva versión en RViz ni Gazebo en el equipo de preparación. La captura enviada por el usuario corresponde al paquete anterior funcionando en RViz.

## Si sigue apareciendo gris después de actualizar

Cierra y vuelve a abrir RViz/Gazebo para recargar las mallas. Comprueba que se usa el workspace recompilado:

```bash
ros2 pkg prefix --share hartech_tools
grep -n 'part_045.dae' "$(ros2 pkg prefix --share hartech_tools)/urdf/parallel_gripper.xacro"
ls "$(ros2 pkg prefix --share hartech_tools)/meshes/part_045.dae"
```

No uses un URDF expandido guardado antes de esta actualización: vuelve a generarlo desde los Xacro nuevos. Los comandos y controladores de apertura no cambian.

Fuente técnica: [Gazebo Classic — Import Meshes](https://classic.gazebosim.org/tutorials?tut=import_mesh), que documenta COLLADA como contenedor de geometría y materiales. Consultada el 14 de octubre de 2026.
