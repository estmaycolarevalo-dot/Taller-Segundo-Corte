# Parte c — Consola de mandos ESP32 para el robot Boston Dynamics Atlas

**Asignatura:** Micros — Universidad Militar Nueva Granada

## Enunciado

> **c.** Desarrollar una consola de mandos con la ESP32 para un movimiento fluido del
> robot Boston Dynamics que permita una movilidad real.
> Repositorio: https://github.com/erwincoumans/pybullet_robots/tree/master

Misma idea que la parte b (consola de 3 pulsadores, movimiento fluido por cinemática
inversa), pero con el robot humanoide **Atlas** (`atlas_v4_with_multisense.urdf`,
incluido en el repositorio de referencia). Con los mismos 3 pulsadores se controlan la
**mano derecha**, la **mano izquierda** y el **cuerpo** (traslación y giro).

## 1. Arquitectura

```
        ESP32                         PC (Python)                       PyBullet
┌──────────────────────┐   Serial   ┌───────────────────────┐   IK   ┌───────────────┐
│  3 pulsadores         │  USB, 115200 baud                  │       │  Atlas         │
│  grupo: mano der /    │ ─────────▶│  atlas_consola.py      │ ─────▶│  2 brazos +    │
│  mano izq / cuerpo    │  "POS3,..."                         │       │  cuerpo        │
└──────────────────────┘            │  - parsea el mensaje   │       └───────────────┘
                                     │  - mueve la base       │
                                     │  - IK de cada mano     │
                                     └───────────────────────┘
```

## 2. Manejo de la consola

| Botón | Pin ESP32 | Función |
|---|---|---|
| SELECCIONAR (toque corto) | GPIO 27 | Cambia el eje activo del grupo actual |
| SELECCIONAR (mantenido > 0.8 s) | GPIO 27 | Cambia el grupo: mano derecha → mano izquierda → cuerpo |
| MENOS (–) | GPIO 26 | Mantenido: disminuye el valor del eje activo |
| MÁS (+) | GPIO 25 | Mantenido: lo aumenta |

| Grupo | Ejes | Rango |
|---|---|---|
| Mano derecha | X, Y, Z | ±0.2 m respecto a su posición inicial |
| Mano izquierda | X, Y, Z | ±0.2 m respecto a su posición inicial |
| Cuerpo | X, Y, GIRO | ±1.5 m, y ±3.14 rad de giro |

El Monitor Serie (o la terminal de Python) muestra el grupo y el eje activos cada vez
que cambian, por ejemplo `Grupo: MANO IZQUIERDA, eje: Z`.

## 3. Análisis del proyecto

**Diferencia clave frente a Baxter:** Atlas es un humanoide de 30 grados de libertad, no
un brazo industrial de base fija. Mover sus manos sin que el resto del cuerpo se
descontrole requiere varias decisiones:

- **Base fija y cuerpo sostenido.** El robot se carga con `useFixedBase=True` y todas
  las articulaciones reciben control de posición hacia su postura inicial. Piernas,
  torso y cuello quedan quietos mientras solo los brazos reciben órdenes nuevas.
- **IK limitada a cada brazo.** `p.calculateInverseKinematics` calcula por defecto una
  solución para las 30 articulaciones. Con el parámetro `jointDamping` se da muy poca
  resistencia a las 7 articulaciones del brazo que se quiere mover y mucha al resto, de
  modo que la solución recae casi por completo sobre ese brazo. Se resuelve una IK por
  cada mano (`r_hand` y `l_hand`).
- **Pose inicial con codos doblados.** En la pose por defecto de Atlas (en T) los brazos
  están completamente estirados: la mano no puede alejarse más del cuerpo y el solver
  de IK queda en una singularidad. Por eso al iniciar se doblan los codos (`ely` y `elx`)
  y las manos quedan al frente, a la altura de los hombros. Desde ahí hay recorrido en
  todas las direcciones; se verificó que ambas manos alcanzan su objetivo en los tres
  ejes dentro de ±0.2 m, que es el límite configurado.
- **Control relativo.** Cada mano se mueve respecto a su posición inicial, no en
  coordenadas absolutas del mundo. Además, los desplazamientos se expresan en el marco
  del cuerpo: si el robot gira o se traslada, las manos lo acompañan.
- **Traslación y giro del cuerpo.** Se implementan moviendo la base fija con
  `p.resetBasePositionAndOrientation` en cada ciclo. Es un desplazamiento cinemático:
  el robot **se desliza**, no camina (las piernas no se mueven). Una marcha real
  requeriría un controlador de locomoción con equilibrio dinámico, fuera del alcance de
  esta práctica.
- **Mismo hardware, protocolo extendido.** La consola física es la misma de la parte b
  (3 pulsadores). El mensaje serial pasa de un solo objetivo a tres, y la pulsación larga
  (que en Baxter abría y cerraba la pinza) ahora cambia de grupo, porque Atlas no tiene
  pinza en este modelo.

## 4. Paso a paso

### 4.1. Entorno de Python

```bash
conda activate drones
pip install pybullet pyserial numpy
```

### 4.2. Modelo de Atlas

Está incluido en la carpeta [`atlas/`](./atlas) (tomada de `pybullet_robots/data/atlas`).
Debe quedar junto a `atlas_consola.py`.

### 4.3. Programar la ESP32

Cargar [`firmware/arduino/esp32_consola_atlas_3botones.ino`](./firmware/arduino/esp32_consola_atlas_3botones.ino)
en el Arduino IDE (placa *ESP32 Dev Module*). Conectar los 3 pulsadores como indica la
tabla de la sección 2, cada uno de su pin a GND, sin resistencias externas.

### 4.4. Ejecutar la simulación

Con la ESP32 conectada y el Monitor Serie **cerrado**, desde esta carpeta:

```bash
python atlas_consola.py
```

## 5. Código con explicación

- **`parsear()`:** valida el mensaje `POS3` (9 valores) y recorta cada uno a su rango.
- **`mapear_joints()`:** crea dos diccionarios (nombre de articulación → índice, nombre
  de link → índice) para localizar las articulaciones de cada brazo y las manos.
- **`construir_damping()` / `accurate_ik()` / `mover_brazo()`:** resuelven la IK de un
  brazo reseteando solo sus 7 articulaciones y luego aplican el resultado con control de
  posición. Se llaman una vez por mano en cada ciclo.
- **Bucle principal:** (1) lee y parsea el serial sin bloquear, (2) mueve la base según
  el grupo CUERPO, (3) calcula el objetivo de cada mano en el marco del cuerpo
  (`base + R · (posición inicial + desplazamiento)`), (4) resuelve la IK de ambas manos
  y (5) actualiza las etiquetas DER (roja) e IZQ (azul) que marcan los objetivos.

## 6. Formato del mensaje serial

```
POS3,<rx>,<ry>,<rz>,<lx>,<ly>,<lz>,<bx>,<by>,<giro>
```

`r` = mano derecha, `l` = mano izquierda (metros, respecto a su posición inicial),
`b` = cuerpo (metros en x e y, y giro en radianes).
Ejemplo: `POS3,0.100,0.000,0.050,-0.100,0.000,0.000,0.500,0.000,0.785`

## 7. Evidencia

Videos del funcionamiento (consola física + simulación de Atlas):

- [Evidencia 1c](https://drive.google.com/file/d/1vqwhFFrqrlTJBy4VgVOPjl4dG4JZd5BO/view?usp=sharing)
- [Evidencia 2c](https://drive.google.com/file/d/18z95IucYVXfa1tptN7XGoLHjXvNHldaT/view?usp=sharing)

## 8. Conclusiones

- El mismo hardware de 3 pulsadores permite controlar tres elementos distintos (dos
  manos y el cuerpo) con un esquema de grupos y ejes, sin añadir componentes.
- Controlar una parte de un robot de muchos grados de libertad exige técnicas que no se
  necesitan en un brazo de base fija: sostener el resto del cuerpo, sesgar la IK hacia
  el miembro deseado y elegir una pose inicial lejos de singularidades.
- La traslación del cuerpo es cinemática (deslizamiento); una locomoción real con
  marcha es una extensión natural para trabajo futuro.

## Repositorio de referencia

- https://github.com/erwincoumans/pybullet_robots
- https://github.com/erwincoumans/pybullet_robots/blob/master/atlas.py
