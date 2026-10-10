# Límites articulares — revisión mecánica V1 en curso

Los montajes J1/J6 añadidos el 15–16/09 tampoco están incluidos en las pruebas históricas de este documento. Sus rotores auxiliares están colocados para la pose guardada, sin relación automática de movimiento. Se conservan los límites originales de las seis juntas y quedan pendientes los barridos con las nuevas carcasas y los arneses completos.

El montaje lateral del encoder J4 y su nueva carcasa inferior se añadieron después de las pruebas aquí registradas. Tampoco quedan cubiertos por esos barridos. El rotor de lectura está colocado para la pose actual y todavía no sigue automáticamente los movimientos de J4.

**Revisión CAD posterior:** se añadieron los encoders J3/J5 después de los barridos descritos aquí. Sus soportes, carcasas y herrajes no están incluidos en esas pruebas. Por petición del usuario se prioriza continuar el CAD y se aplazan nuevos barridos; los límites se conservan provisionalmente.

El 13/09 a las 22:48 se completaron las 262 muestras de `_ENCODER_J2_L1_L2`, sin colisiones. Se comprobaron L1 y L2 contra los restantes eslabones e incluyendo el par L1–L2. Cubre el soporte, carcasa y portaimán del primer encoder J2, además de sus fijaciones; las colisiones internas se comprobaron aparte en horizontal. La propagación de matrices se contrastó de nuevo con las seis juntas nativas (error máximo 1.15×10⁻¹³). Sigue siendo un barrido discreto sin validación de los bucles flexibles pendientes.

El 13/09 a las 12:26:59 terminó el barrido adicional `_TENSOR_J4_L3`: las mismas 262 configuraciones, verificando el eslabón modificado L3 contra los otros seis, sin colisiones. Las juntas se restauraron y el conjunto se guardó en HOME. La geometría interna de L3 se había comprobado aparte. Esto añade evidencia sobre el tensor J4 al barrido general `_POST_TOPE`; no incluye encoders incorporados posteriormente ni demuestra continuidad entre muestras.

Actualizado: 13 de septiembre de 2026, después del tensor J4 y del primer encoder J2. Ángulos en grados, respecto a la pose horizontal del ensamblaje. Se conservan las seis juntas originales; sus límites son provisionales mientras falten arneses, sensores y los restantes topes físicos.

| Eje | Límite CAD inferior | Límite CAD superior | Fundamento y situación |
|---|---:|---:|---|
| J1 | −160 | +160 | Rango previo; barrido individual sin choques en las muestras. Falta justificarlo con torsión del arnés y tope. |
| J2 | −95 | **+5** | Superior corregido por choque medido del soporte J3 contra la mejilla J2. Tope positivo incorporado. Inferior aún pendiente de tope y combinación completa de movimientos. |
| J3 | −110 | +110 | Rango previo; muestras individuales y trayectorias indicadas sin colisiones propias. Falta arnés y tope. |
| J4 | −150 | +150 | Rango previo; falta verificar torsión dinámica del arnés y sensor. |
| J5 | −70 | +70 | Rango previo; falta bucle flexible, sensor y tope. |
| J6 | −160 | +160 | Rango previo; falta cableado a brida, sensor y tope. |

## Barrido anterior que motivó corregir J2

`BARRIDO_GEOMETRICO_V1.csv` y `BARRIDO_GEOMETRICO_INTERFERENCIAS_V1.csv` registran 264 muestras calculadas desde matrices y ejes medidos del ensamblaje. Se contrastó esa propagación con los seis movimientos nativos de 15°; error máximo de matriz 2.67×10⁻¹³ en esa ejecución. Después se colocaron los cuerpos rígidos con juntas temporalmente suprimidas, se ejecutó `AnalyzeInterference` y se restauraron juntas, fijaciones y pose HOME. El registro `GEOSWEEP_COMPLETE samples=264` confirma su terminación el 12/09 a las 15:38.

Se revisó HOME, cada articulación hacia ambos extremos desde HOME y dos trayectorias coordinadas para cada par J2/J3, J3/J4, J4/J5 y J5/J6. El incremento fue de hasta 10° por articulación. Son muestras progresivas de sólidos rígidos: no prueban ausencia de choques entre muestras, todas las combinaciones articulares ni comportamiento de cables flexibles ausentes.

Se detectaron 7 configuraciones con colisión, con 18 pares de sólidos en total. Dos estaban en el recorrido positivo de J2, a +15.625° y +25°; cinco en la trayectoria J2/J3 que terminaba en (+25°,−110°). El choque común fue `SOPORTE_MOTOR_J3` contra `MEJILLA_J2_36`. Con J2=+25°, J3=+80° también la muñeca y varias piezas alcanzaron la base. El rango anterior de J2 hasta +25° queda invalidado por estos resultados.

Ese barrido precede a la tapa exterior de J4, la abrazadera de guía y el tope J2. Sus CSV se conservan como evidencia del hallazgo; las trayectorias se volvieron a comprobar con esos herrajes y con el límite superior de J2 reducido a +5°, como se registra a continuación. Las seis poses habituales también se repitieron con esos herrajes: consultar sus CSV y fechas.

## Barrido terminado con el tope J2

`BARRIDO_GEOMETRICO_V1_POST_TOPE.csv` contiene **262 muestras únicas, numeradas de 0 a 261, todas con cero interferencias y volumen cruzado cero**. `BARRIDO_GEOMETRICO_INTERFERENCIAS_V1_POST_TOPE.csv` contiene solamente su cabecera. El máximo ángulo J2 muestreado es +5°. El registro `GEOSWEEP_COMPLETE revision=_POST_TOPE samples=262` confirma la terminación el 12/09 a las 23:17:39 y la restauración de las juntas nativas.

Se volvieron a recorrer HOME, los extremos individuales y las trayectorias coordinadas indicadas, con la tapa exterior J4, la abrazadera de guía y el tope J2 incorporados. El resultado cubre esas posiciones discretas de sólidos rígidos; no demuestra ausencia de choques entre muestras, todas las combinaciones articulares ni comportamiento de cables flexibles ausentes. Es anterior a los siguientes cambios de tensado: cualquier herraje o desplazamiento posterior exige su propia comprobación.

## Medición fina del contacto J2/J3 y tope incorporado

`HOLGURA_J2_SOPORTE_J3_V1.csv` contiene mediciones nativas del par de piezas antes de añadir el tope:

| J2 | Separación mínima mm | Volumen cruzado mm³ |
|---|---:|---:|
| +4° | 2.802560 | 0 |
| +5° | 2.221700 | 0 |
| +6° | 1.628588 | 0 |
| +7° | 1.023404 | 0 |
| +8° | 0.406334 | 0 |
| +8.5° | 0.093400 | 0 |
| +8.75° | 0 | 0.109504 |

El inicio del choque queda acotado entre +8.5° y +8.75°. Se eligió un límite CAD de **+5°**, y contacto geométrico nominal del tope a **+5.25°**. No se modifica alcance, motores, relaciones ni carga objetivo.

`TOPE_POSITIVO_J2_5_25DEG_V1.ipt` es una placa de acero apoyada contra el canto X38 de la mejilla positiva, Y36–44 y Z40–66 en L1. Su cara de contacto está en X40.0745633; espesor 2.0745633 mm. Esa posición sale de la esquina X43, Z−30 del pie activo del soporte de motor J3: X(q)=43 cos(q)−30 sin(q). Dos M3×10, centros Y40 y Z45/53, sujetan el tope con aproximadamente 7.925 mm de rosca útil. La carga normal se transmite por apoyo contra la mejilla.

La separación angular de 0.25° entre límite CAD y contacto es nominal; **falta tolerancia final, calibración del home y distancia de frenado**. El tope sirve como fin mecánico positivo, no como zona de operación continua. Su resistencia al impacto y la del pie de aluminio que lo contacta no están calculadas todavía.

La prueba de contacto controlada terminó y se registra en `PRUEBA_TOPE_J2_V1.csv`: separación de 0.147136 mm a +5°, contacto sin volumen cruzado a +5.25° y cruce de 0.917115 mm³ a +5.5° solo durante diagnóstico. La junta se devuelve a −95°/+5° y el conjunto se guarda en HOME. No se debe ejecutar el diagnóstico antiguo que gira hasta +10° con el tope instalado.

La prueba individual de movilidad de J2 se realiza ahora a −15° desde cero para respetar el límite positivo; los otros cinco ejes mantienen +15°. La magnitud de movimiento y el origen de giro se vuelven a medir desde las matrices del ensamblaje.
