# Corrección de propiedades físicas y resultados de carga

Los resultados calculados a partir de `MASAS_CAD_CIERRE_V1.csv` quedan retirados como resultados horizontales. Ese archivo describía la pose HOME, pero el cálculo aplicaba brazos de palanca y ejes de la pose horizontal. En particular, **0.251 kg no es una capacidad válida del robot**, ni una carga útil certificada. Tampoco deben reutilizarse los torques anteriores de 9.859 Nm para J2 y 3.837 Nm para J3 como resultados de la geometría final.

Se encontraron además dos errores en la primera exportación de inercias: las coordenadas en centímetros se multiplicaban por 0.1 en lugar de 0.01 para obtener metros, y se intercambiaban los argumentos Iyz/Ixz. El archivo de inercias anterior a la exportación V2 no es apto para URDF.

## Exportación corregida

`08_Scripts/RobotPhysicalExportV2.cs` extrae masa, centro de masa, momentos XYZ y momentos principales de la definición local de cada uno de los siete subconjuntos. No mueve ni modifica las juntas. Comprueba los centros de los sketches de entrada y salida de las piezas `REFERENCIAS`; transforma el centro local a esos marcos y acumula las separaciones nominales de la cadena. Así reconstruye una pose horizontal matemática independiente de HOME.

- `MASAS_CAD_FINAL_HORIZONTAL.csv`: masa y centro de masa de L0–L6 en el marco horizontal nominal, en kg y mm; incluye la inercia propia Izz en kg·m² para J1.
- `PROPIEDADES_INERCIALES.csv`: masa, centro de masa en metros y tensor en kg·m² respecto del centro de masa, con ejes paralelos al marco nominal de cada link. Los elementos cruzados usan la convención del tensor de cuerpo rígido, es decir, integral negativa.
- `INERCIAS_API_AUDITORIA_FINAL.csv`: valores API sin cambio de signo, momentos principales y multiplicador aplicado a los elementos cruzados.

La transformación de centímetros a metros es 0.01; la de kg·cm² a kg·m² es 0.0001. La llamada documentada de Inventor recibe **Ixx, Iyy, Izz, Ixy, Iyz, Ixz**. El método entrega momentos respecto al centro de gravedad, no respecto al origen del link; por tanto no se resta un término de ejes paralelos al tensor local. [Autodesk: XYZMomentsOfInertia](https://help.autodesk.com/cloudhelp/2022/ENU/Inventor-API/files/MassProperties_XYZMomentsOfInertia.htm).

La interfaz gráfica de Inventor permite mostrar integral positiva o negativa; la segunda corresponde al tensor rígido. Para no depender de esa preferencia, V2 contrasta ambas convenciones de signo contra la traza, la norma y el determinante definidos por los momentos principales. Exige un resultado único y consistente en el conjunto de eslabones. Si no puede resolverlo, falla sin publicar una nueva exportación. Esta comprobación es de consistencia de los datos, no un ensayo mecánico del robot. [Autodesk: opciones generales, propiedades físicas](https://help.autodesk.com/cloudhelp/2015/ENU/Inventor-Help/files/GUID-721143C6-67AA-4EDA-B98A-C8E45955D332.htm).

## Recalcular después de la corrección CAD

1. Ejecutar la exportación `v2physical` cuando estén guardadas las correcciones de montaje.
2. Ejecutar `py -3.13 01_Calculos/actualizar_torques_cad.py`.
3. Usar únicamente `TORQUES_CAD.json` y `CALCULO_TORQUES.md` regenerados con la identificación UTC de esa exportación.

El cálculo exige el marco `HORIZONTAL_NOMINAL_V2`, los siete links ordenados y un identificador coincidente entre las masas y las inercias. No acepta el archivo HOME ni recupera silenciosamente datos anteriores. J1 incorpora ahora las inercias propias de los eslabones además de sus separaciones de masa; sigue excluyendo inercias reflejadas de rotores y acoplamientos dinámicos.

El criterio de motor usado por el estudio sigue siendo una fracción supuesta del par de bloqueo, con eficiencia supuesta. No es un par continuo garantizado. El resultado final indica expresamente si los 200 g de objeto más 100 g de herramienta satisfacen ese criterio; si alguna demanda lo supera, no se declara aprobado el objetivo de carga. Ningún umbral de este cálculo equivale a una capacidad nominal certificada.
