# Decisiones de diseño — robot serial 6R

Última ampliación CAD: encoders externos J3 y J5 añadidos a los eslabones correspondientes, cada uno con 29 componentes entre receptor, portaimán, placa, carcasa y herrajes. La selección de motor, transmisión, juntas y alcance se mantiene. Los datos numéricos de la revisión J2 que sigue son anteriores a esta ampliación; pendientes los nuevos cálculos y barridos, aplazados para priorizar el modelado según la petición del usuario.

Proyecto: `D:\Utec\GPT6\CHAT_6.ipj`. Modelo principal: `04_Ensamblajes\ROBOT_6DOF.iam`.

Actualización 13/09 — primer encoder externo AS5048A en J2: receptor fijo a L1 y portaimán coaxial en L2, con carcasa desmontable y entrehierro CAD de 1.3 mm comprobado en cuatro posiciones. Se preservan las seis juntas, alcance y transmisiones. Masa conjunta 7.671411 kg; la documentación de sensor identifica las dimensiones de PCB/conector todavía supuestas. El paso de tornillos M3 en la mejilla se cambió a pretaladro pasante para evitar fondo con los M3×8. No equivale a validación magnética ni retención axial completa; detalle en `INTEGRACION_AS5048A.md`.

## Alcance de la revisión

Primera iteración de ingeniería mecánica con piezas nativas editables, motores STEP oficiales y cadena articulada en Inventor. El cierre de tolerancias, selección comercial de transmisiones, retención y uniones de fabricación requiere una revisión posterior; no emitir estas piezas como planos aprobados de taller.

## Arquitectura y cotas

- Siete cuerpos rígidos unidos por seis articulaciones de revolución: base fija; hombro; brazo; antebrazo; roll de muñeca; pitch; brida.
- Ejes de la pose horizontal: J1 Z; J2 Y; J3 Y; J4 X; J5 Y; J6 X.
- Altura J1 178 mm y J2 273 mm sobre la mesa. J2–J3 160 mm; J3–J4 130 mm; J4–J5 45 mm; J5–J6 104 mm. Fin de brida a 448 mm desde J2. La revisión V05 trasladó J4 10 mm hacia el codo para separar la transmisión J5 del soporte fijo al girar la muñeca.
- La muñeca es más larga que la estimación inicial para alojar el motor Pololu de 66.5/67 mm, el eje de 12.5 mm, acople y apoyo independiente. Se mantiene el alcance dentro del intervalo inicial 350–450 mm.
- Objeto objetivo 200 g y herramienta 100 g. Sus CG se asumen a 20 y 15 mm de la cara de brida; excentricidad radial de cálculo hasta 20 mm. No es payload certificado.

## Componentes verificados

Se descargaron e importaron los STEP originales goBILDA 5203-2402-0071 y Pololu 25D 34–47:1 con encoder. Las masas se sobrescribieron con 486 g y 101 g; asignar densidad homogénea de metal al interior del motor habría sido incorrecto. La localización de CG sigue derivándose de la geometría, no de un ensayo.

Datos, cotas, límites y enlaces en [MOTORES_FUENTES.md](MOTORES_FUENTES.md). Rodamientos de contorno verificado 6001 (12×28×8), 6000 (10×26×8), 61804 (20×32×7). Se modelan como anillos para montaje, con masa comercial sobrescrita: 22, 19 y 20 g respectivamente; no se simulan bolas, jaulas ni contactos internos.

## Dimensionamiento y transmisiones

1. El análisis previo, guardado en `01_Calculos`, descartó accionamiento directo para hombro/codo. El criterio de selección usa 20 % del bloqueo, eficiencia adicional 0.85 y reserva de carga 1.5. Esta fracción no es par continuo garantizado.
2. J2 empezó con 5:1 y se elevó a 6:1 y finalmente 7:1 al incorporar masas CAD y reservas. J3 pasó de 3:1 a 4:1 en V06 para absorber el peso de las carcasas y la envolvente gravitatoria de distintas configuraciones. J4/J5 pasaron de 3:1 a 4:1. J1 3:1; J6 1:1 mediante acople.
3. Transmisiones de disposición HTD paso 3 mm: 20/60 J1, 20/140 J2, 20/80 J3, J4 y J5. Anchos de estudio 15 mm J2/J3 y 9 mm restantes. Son elecciones de diseño; faltan referencia de fabricante, capacidad, pretensión, longitud comercial y ajuste por ranuras/tensor.
4. Las poleas son envolventes al diámetro primitivo, algunas aligeradas; no tienen dentado HTD de fabricación. Las correas tienen geometría de tangentes y grosor de envolvente. No imprimirlas ni mecanizarlas directamente.
5. J2/J3 tienen ejes intermedios apoyados para que el esfuerzo de correa no recaiga directamente en el eje del motor. Los ejes principales tienen apoyos separados. El motor J3 se coloca a 67 mm de J2; se desplazó desde 38, luego 62 mm para librar la mejilla del hombro. La polea J3 de 80 dientes se aligera con cuatro huecos Ø17 sobre radio 25 mm.

## Estructura y correcciones

- Placas y soportes de aluminio, densidad nominal 2700 kg/m³. Ejes de acero 7850 kg/m³, salvo husillo hueco de J4 aligerado en aluminio.
- Se corrigieron asientos de rodamiento, cubos que penetraban placas, separadores de base que cruzaban la correa, interferencias de los soportes de motores y longitud de las placas del antebrazo.
- Los soportes de J4 se ampliaron para dar paso a la polea mayor y después se aligeraron mediante ventanas y nervios. Los asientos conservan el ancho requerido por el rodamiento.
- El motor J5 se trasladó al lado distal de su correa, ahora a X95 mm respecto de J4, manteniendo 50 mm entre centros. Así no ocupa el volumen axial de los soportes fijos de J4 cuando gira el roll. Se amplía una mejilla de la horquilla para soportarlo.
- J6 incorpora un segundo apoyo 61804 en el acople Ø20, además del 6000 delantero. Se separa el apoyo de la herramienta del eje de motor; la retención axial y los ajustes siguen pendientes.
- V06 corrige el pie de los apoyos de entrada J3: integra 3 mm de solape con el anillo, eliminando la unión solo tangente de V05.
- Se mantienen versiones anteriores para trazabilidad. Solo las piezas referenciadas por el ensamblaje actual definen la revisión activa. Consultar el inventario de archivos activos al exportar.

## Carcasas V06

Se añaden piezas IPT propias de protección: caja ventilada de base y tapa separada, guardas de las cinco transmisiones, paneles de brazo/antebrazo y fundas ventiladas de los motores J2–J6. Las guardas de correa son capuchones de una pieza, desmontables como unidad; su frente y pared no son dos piezas separables.

Material de cálculo: plástico nominal de 1.25 g/cm³. Es una hipótesis para dimensionar masa; seleccionar y ensayar el material de impresión antes de fabricar. Pared base 2 mm, tapa 1.6 mm, paredes secundarias 1.2 mm y frentes de las guardas distales 1 mm. Las cubiertas no sustituyen las placas de aluminio ni los apoyos de las articulaciones.

Los CSV de masa incluyen las cubiertas como piezas físicas. Las aberturas mantienen ventilación, salida de conectores, paso de ejes y acceso a apoyos. No constituyen un cerramiento estanco ni un resguardo industrial certificado. Los espesores delgados requieren comprobar impresión, rigidez local y vibración; el CAD representa material macizo, sin descontar relleno de laminador.

Se incluyen orejas inferiores de base, tetones superiores para la tapa y fijaciones de paneles/fundas con taladros correspondientes en sus soportes. Los tornillos, insertos o tuercas definitivos y el anclaje detallado de cada guarda de correa deben cerrarse antes de liberar fabricación.

## Ensamblaje y validación

- Cada subconjunto es rígido internamente mediante fijación de sus ocurrencias. Las seis uniones nativas rotacionales están en el ensamblaje principal; solo la base global queda fijada.
- Las referencias son pequeños datums excluidos del BOM. Se excluyen sus volúmenes de la revisión de interferencias, pues no son piezas físicas.
- La primera ruta de uniones basada solo en círculos no conservaba correctamente orientación y origen. Se corrigió usando caras planas de datums circulares, intención del centro del borde y normales explícitas. Se verificaron matrices y coordenadas; no se aceptó únicamente el estado `UpToDate` del solver.
- `PRUEBA_MOVILIDAD.csv` registra ensayos reales de 15° por eje, con los demás temporalmente bloqueados, y errores de pivote/alineación. Los bloqueos se retiran al terminar.
- Los CSV de interferencias proceden de `AnalyzeInterference`. Las pruebas discretas no certifican todo el espacio continuo ni combinaciones arbitrarias de límites.

## Pendientes para liberación a fabricación

- Detalle de transmisión de torque en cubos y ejes: abrazaderas, chavetas o pasadores; acoples REX definitivos y tornillos de apriete. Los taladros de reserva de 9.2 mm de los acoples principales son envolventes, no perfiles REX aprobados.
- Retención axial completa de ejes/rodamientos y ajustes según catálogo: hombros, tapas, anillos y precargas. No se han cerrado tolerancias H7/g6 ni su conveniencia para cada asiento.
- Tornillería estructural completa, roscas, longitudes de penetración, acceso de herramienta, tensores de correa y topes mecánicos.
- Soportes definitivos de encoder absoluto, imanes, fines de carrera y recorridos de cables. La integración mecánica de estos elementos requiere sus referencias y huecos finales.
- Validación de rigidez de las uniones, fatiga, fricción, holguras y temperatura de motores. La aproximación de vigas no sustituye FEA o ensayos.
- Cerrar retención axial de los dos apoyos de J6 y verificar el momento máximo de herramienta.

No se desarrollaron electrónica, PCB, control, cinemática inversa ni visión, conforme al alcance solicitado.

## Inicio del cierre mecánico V1.0

La solicitud de cierre amplía el trabajo de detalle. El registro `CIERRE_V1_EN_CURSO.md` documenta las modificaciones actuales, el respaldo, las convenciones de tornillería y los pendientes. La auditoría API `GEOMETRIA_OCURRENCIAS_V1.csv` identifica las piezas activas y sus posiciones en cada eslabón. No se modificaron las seis juntas funcionales ni sus datums.

La prueba de brida J6 enteramente de acero deja solo 0.0037 Nm de diferencia en J4 bajo el criterio provisional. Se sustituye por unión atornillada de brida de aluminio a eje de acero para recuperar margen sin variar motores, reducciones, alcance o carga objetivo. La siguiente validación debe usar esta revisión ligera y el paso de cableado, no los resultados de la prueba más pesada.

## Retención, guía y límite J2 — 12 de septiembre de 2026

Se ejecutaron en las revisiones activas la retención delantera exterior de J4 con tapa y hombro, apoyo posterior deslizante, arandela bajo el circlip y cabezas de brida embutidas. La guía curva recibe collar y abrazadera partida con cuatro tornillos; el vaciado de sus pies sigue la trayectoria del cable. Detalles en `RETENCION_J4_V1.md` y `CABLEADO_MECANICO.md`.

Un barrido completo de 264 muestras detectó 7 configuraciones con choques, principalmente entre la mejilla J2 positiva y el pie del soporte de motor J3. La medición directa sitúa el inicio entre +8.5° y +8.75°. Se cambió únicamente el límite positivo de J2, de +25° a +5°, y se incorporó tope atornillado para contacto a +5.25°. Las seis juntas se conservan; la prueba J2 pasa a −15°. No se reduce ni cambia la geometría de alcance. Las seis poses habituales siguen dentro de los límites. Ver `LIMITES_ARTICULARES.md` para evidencia, limitaciones y nuevas verificaciones.
# 13/09/2026 — carro tensor J4 sobre el ensamblaje activo

Se conserva la reducción 20/80, paso 3 mm, ancho 9 mm, alcance, carga y las seis juntas originales. La distancia entre centros se ajusta a aproximadamente 60.592 mm para correa candidata OMEGA 285 3M 9. Se integra carro de motor, ranuras Z56–63, bloqueo lateral con tuercas, puente superior y tornillo M3×20 de tracción. La carcasa V06 acompaña el carro con desplazamiento X−3 mm y holgura de conector corregida. Los soportes de rodamiento permanecen en sus posiciones.

El patrón de fijación del motor se obtuvo de su STEP real: en el marco L3 los agujeros están en Y0/Z=C±8.5, corrigiendo la orientación usada en el soporte anterior. Los M3×6 con arandela de 0.5 y placa de 4 penetran 1.5 mm, respetando el máximo de 2 mm comunicado por Pololu. Los tornillos del motor se montan antes de la polea; el bloqueo del carro se accede por sus lados y por nuevas ventanas en la placa delantera. El ajustador se sitúa en Y36 para no cruzar la tuerca de bloqueo Y30.

Respaldo previo en `09_Respaldos/TENSADO_J4_V1`. Las revisiones de interferencia, movimiento, carrera, masa y torque se repiten después de corregir el carro. El barrido de 262 muestras `_POST_TOPE` permanece como evidencia anterior al tensor y no debe sobrescribirse. Las comprobaciones posteriores usarán sufijo `_TENSOR_J4`. La tensión de montaje, capacidad radial del motor, perfil/pestañas comerciales y resistencia del conjunto todavía requieren cierre; no se aumenta la carga útil.
