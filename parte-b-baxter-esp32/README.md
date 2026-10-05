# Parte b — Consola de mandos ESP32 para el robot Baxter

**Asignatura:** Micros — Universidad Militar Nueva Granada

## Enunciado

> **b.** Desarrollar una consola de mandos con la ESP32 para un movimiento fluido del
> robot Baxter que permita una movilidad real de brazos y posicionamiento, además de que
> el robot pueda coger y mover un objeto.
> Repositorio: https://github.com/erwincoumans/pybullet_robots ·
> https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py

La parte c usa la misma consola de 3 pulsadores y la misma lógica de control, pero con
el robot Boston Dynamics Atlas en vez de Baxter — ver [`../parte-c-atlas-esp32/`](../parte-c-atlas-esp32).

## 1. Arquitectura

El sistema se divide en tres capas: **entrada física** (los pulsadores en la ESP32),
**puente de comunicación** (serial USB) y **ejecución física** (el brazo Baxter simulado
en PyBullet, con cinemática inversa).

```
        ESP32                         PC (Python)                       PyBullet
┌──────────────────────┐   Serial   ┌───────────────────────┐   IK   ┌───────────────┐
│  3 pulsadores (jog)   │  USB, 115200 baud                  │       │  Brazo Baxter  │
│  SELECCIONAR: cambia  │ ─────────▶│  baxter_consola.py     │ ─────▶│  + pinza       │
│  eje / abre-cierra    │  "POS,x,y,z,grip"                  │       │  + cubo objeto │
│  pinza                │           │  - parsea el mensaje   │       │                │
│  MENOS/MAS: mueven el │           │  - calcula IK hacia    │       │                │
│  eje activo (continuo)│           │    (x,y,z)             │       │                │
└──────────────────────┘            │  - abre/cierra pinza   │       └───────────────┘
                                     │  - simula el agarre    │
                                     │    con un constraint   │
                                     │    rígido temporal      │
                                     └───────────────────────┘
```

**Componentes:**

| Capa | Elemento | Responsabilidad |
|---|---|---|
| Entrada | ESP32 (`firmware/arduino/esp32_consola_baxter_3botones.ino`) | Leer 3 pulsadores, mantener la posición objetivo (x, y, z) y el estado de la pinza, y transmitirlos ~20 veces por segundo. |
| Comunicación | Puerto serie USB (115200 baudios) | Transporta el mensaje de texto plano `POS,<x>,<y>,<z>,<grip>`. |
| Ejecución | `baxter_consola.py` (PyBullet) | Recibe cada mensaje, calcula la cinemática inversa (IK) del brazo de Baxter hacia esa posición, mueve los motores y controla la pinza. |

## 2. Análisis del proyecto

**Problema a resolver:** lograr que una persona mueva el efector final del brazo Baxter
de forma fluida y en tiempo real usando solo hardware simple (3 botones), y que ese brazo
pueda además tomar y desplazar un objeto — sin necesitar un joystick ni hardware costoso.

**Decisiones de diseño clave:**

- **Control tipo "jog" en vez de joystick.** Con solo 3 pulsadores (SELECCIONAR, MENOS,
  MÁS) se emula el panel de control de una impresora 3D: se elige un eje activo (X, Y o
  Z) y se avanza/retrocede en él mientras se mantiene presionado el botón. Esto evita
  depender de un módulo analógico adicional y simplifica el cableado (pull-up interno,
  sin resistencias externas).
- **Movimiento continuo, no por pasos discretos.** El ESP32 no envía una sola coordenada
  por pulsación; envía la posición actualizada ~20 veces por segundo mientras el botón
  está presionado, lo que produce un movimiento fluido del brazo en vez de saltos.
- **Cinemática inversa (IK) en tiempo real.** En lugar de precalcular trayectorias,
  `baxter_consola.py` llama a `p.calculateInverseKinematics` en cada ciclo de simulación,
  igual que hace `baxter_ik_demo.py` del repositorio de referencia. Esto permite que el
  brazo persiga cualquier posición objetivo que llegue por serial, en cualquier momento.
- **Agarre robusto del objeto.** El brazo se mueve por IK "cinemática" (se teletransportan
  los ángulos de las articulaciones), así que la fricción real entre los dedos y el cubo
  no basta para sostenerlo mientras se desplaza. Se resuelve creando un **acople rígido
  temporal** (`p.createConstraint`) entre la pinza y el cubo cuando la pinza se cierra
  estando a menos de 6 cm de él; se libera en cuanto se vuelve a abrir. Es la técnica
  estándar en PyBullet para simular un agarre confiable en demos de este tipo.
- **Límites de seguridad.** Tanto en el firmware (constantes `RANGO_XY`, `Z_MIN`, `Z_MAX`)
  como en el script de Python (`np.clip`) se recorta la posición recibida, para que un
  error de lectura o un valor fuera de rango no mande al brazo a una posición inválida.

## 3. Paso a paso

### 3.1. Preparar el entorno de Python

```bash
conda create -n drones python=3.12 -y
conda activate drones
pip install --upgrade pip
pip install pybullet pyserial numpy
conda install git -y
```

### 3.2. Clonar el repositorio con los modelos del robot

```bash
git clone https://github.com/erwincoumans/pybullet_robots.git
```

Copiar `baxter_consola.py` **dentro** de la carpeta `pybullet_robots` (el script necesita
la ruta relativa `baxter_common/baxter_description/urdf/toms_baxter.urdf` que trae ese
repositorio).

### 3.3. Programar la ESP32

Abrir `firmware/arduino/esp32_consola_baxter_3botones.ino` en el Arduino IDE, seleccionar
la placa *ESP32 Dev Module* y el puerto correspondiente, y subir el sketch. Conectar 3
pulsadores según la siguiente tabla (cada uno de su pin a GND, sin resistencias externas):

| Botón | Pin ESP32 | Función |
|---|---|---|
| SELECCIONAR | GPIO 27 | Corta: cambia el eje activo (X → Y → Z → X...). Larga (>0.8 s): abre/cierra la pinza. |
| MENOS (–) | GPIO 26 | Mantenido: disminuye la coordenada del eje activo, de forma continua. |
| MÁS (+) | GPIO 25 | Mantenido: la aumenta, de forma continua. |

### 3.4. Ejecutar la simulación

Con el ESP32 conectado, el sketch cargado, el Monitor Serie **cerrado** (para liberar el
puerto), y el entorno `drones` activo:

```bash
cd pybullet_robots
python baxter_consola.py
```

Se abre la ventana de PyBullet con Baxter y un cubo pequeño sobre una plataforma gris,
cerca de donde arranca el brazo. Con SELECCIONAR eliges el eje, con MENOS/MÁS te
desplazas por él, y con una pulsación larga de SELECCIONAR abres o cierras la pinza.

## 4. Código con explicación

### 4.1. Firmware ESP32 — [`firmware/arduino/esp32_consola_baxter_3botones.ino`](./firmware/arduino/esp32_consola_baxter_3botones.ino)

- **Antirrebote + pulsación larga (líneas 39-83):** el botón SELECCIONAR se lee con
  antirrebote por tiempo (`ANTIRREBOTE_MS`) y se mide cuánto dura presionado; menos de
  800 ms cambia de eje, más de 800 ms alterna la pinza.
- **Movimiento continuo (líneas 85-100):** cada 50 ms (~20 Hz) se revisa si MENOS o MÁS
  están presionados y se suma/resta un `delta` proporcional a `VELOCIDAD`, generando un
  movimiento fluido en vez de saltos discretos. La posición se recorta con `constrain()`
  a los límites de seguridad.
- **Envío del mensaje (línea 99):** `Serial.printf("POS,%.3f,%.3f,%.3f,%d\n", ...)` —
  mismo formato que espera `baxter_consola.py`.

### 4.2. Puente Python — [`baxter_consola.py`](./baxter_consola.py)

- **`abrir_serial()`:** detecta automáticamente el puerto del ESP32 buscando
  descripciones típicas de sus chips USB-serial (CP210, CH340, etc.), para no tener que
  configurar el puerto a mano cada vez.
- **`parsear()`:** valida y recorta cada línea recibida (`POS,x,y,z,grip`) antes de
  usarla como objetivo, protegiendo contra líneas corruptas o fuera de rango.
- **`accurate_ik()`:** versión simplificada del `accurateIK` de `baxter_ik_demo.py`, con
  pocas iteraciones para no bloquear el lazo en tiempo real. Excluye las articulaciones
  de la pinza del reset de posición, para no perder el cierre aplicado por
  `set_gripper()`.
- **`set_gripper()`:** mueve los dedos de la pinza a la posición abierta o cerrada con
  una fuerza suficiente (`GRIP_FUERZA`) para sostener el objeto.
- **Bucle principal (`main()`):** en cada iteración (1) lee y parsea el serial sin
  bloquear, (2) calcula IK hacia el objetivo y mueve los motores, y (3) gestiona el
  acople rígido temporal (`p.createConstraint` / `p.removeConstraint`) entre la pinza y
  el cubo, según la distancia entre ambos y el estado de la pinza.

## 5. Formato del mensaje serial

```
POS,<x>,<y>,<z>,<grip>      grip: 1 = abierta, 0 = cerrada
```

Ejemplo: `POS,0.200,0.000,-0.100,1` → mover el efector final a (0.2, 0.0, -0.1) m, con la
pinza abierta.

## 6. Evidencia

_Video de la simulación funcionando: agregar aquí el enlace o adjuntar el archivo en
[`evidencias/`](./evidencias)._

## 7. Conclusiones

- Un control tipo "jog" con solo 3 pulsadores es suficiente para lograr una movilidad
  fluida y real del efector final de Baxter, sin necesitar joystick ni hardware
  analógico adicional.
- La cinemática inversa recalculada en cada ciclo permite que el brazo persiga cualquier
  objetivo recibido por serial en tiempo real, replicando el comportamiento de
  `baxter_ik_demo.py` pero gobernado por hardware externo.
- Simular el agarre con un acople rígido temporal resuelve la limitación de que un brazo
  movido por IK cinemática no genera fricción física suficiente para sostener un objeto.

## Repositorio de referencia

- https://github.com/erwincoumans/pybullet_robots
- https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py
