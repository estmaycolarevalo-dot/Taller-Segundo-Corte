# Taller Segundo Corte — Práctica Real-to-Sim con ESP32

**Asignatura:** Micros — Universidad Militar Nueva Granada

## Enunciado

> Teniendo presente lo aprendido en clase desarrollar la siguiente práctica denominada
> real-to-sim donde se debe desarrollar lo siguiente:
>
> **a.** Mover los drones de un lugar A a un lugar B y a un lugar C, teniendo presente que
> el control se gestionará desde la ESP32.
> Repositorio: https://github.com/utiasDSL/gym-pybullet-drones
>
> **b.** Desarrollar una consola de mandos con la ESP32 para un movimiento fluido del
> robot Baxter que permita una movilidad real de brazos y posicionamiento, además de que
> el robot pueda coger y mover un objeto.
> Repositorio: https://github.com/erwincoumans/pybullet_robots ·
> https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py
>
> **c.** Desarrollar una consola de mandos con la ESP32 para un movimiento fluido del
> robot Boston Dynamics que permita una movilidad real.
> Repositorio: https://github.com/erwincoumans/pybullet_robots/tree/master
>
> El taller se debe desarrollar en un repositorio de GitHub denominado
> **"Taller Segundo Corte"**, donde desarrollarán un `README.md` mostrando la
> arquitectura, el análisis desarrollado del proyecto y el paso a paso a seguir, y donde
> pueden adjuntar videos del funcionamiento y los códigos con su respectiva explicación.

Las tres partes comparten la misma idea central: un microcontrolador **ESP32** actúa
como consola de mando física (con pulsadores) que gobierna, por comunicación serial, un
robot simulado en **PyBullet** en tiempo real.

## Contenido del repositorio

| Carpeta | Parte del enunciado | Qué contiene |
|---|---|---|
| [`parte-a-drones-esp32/`](./parte-a-drones-esp32) | a) Mover drones A → B → C con ESP32 | Simulación de un cuadricóptero (`gym-pybullet-drones`) que recorre 3 puntos de ruta, avanzados con el botón BOOT de la ESP32. |
| [`parte-b-baxter-esp32/`](./parte-b-baxter-esp32) | b) Consola de mandos Baxter | Consola con 3 pulsadores que mueve el brazo Baxter en tiempo real (cinemática inversa) y controla la pinza para coger y mover un objeto. |
| [`parte-c-atlas-esp32/`](./parte-c-atlas-esp32) | c) Consola de mandos Boston Dynamics Atlas | La misma consola de 3 pulsadores de la parte b, ahora con grupos (mano derecha, mano izquierda y cuerpo) para mover ambas manos y trasladar/girar al robot humanoide Atlas. |

Cada subcarpeta tiene su propio `README.md` con la arquitectura, el análisis y el paso a
paso detallado de esa parte.

## Arquitectura general

Los tres puntos siguen el mismo patrón: **ESP32 (entrada/decisión) → Serial USB
(comunicación) → PyBullet (ejecución física)**.

```
        ESP32                          PC (Python)                    PyBullet
┌───────────────────┐    Serial USB   ┌─────────────────────┐        ┌────────────────┐
│  Pulsadores /      │ ───────────────▶│  Script de control  │ ──────▶│  Robot          │
│  botón BOOT        │  texto plano    │  (interpreta el     │  IK /  │  simulado       │
│  (decide / mueve)  │                 │  mensaje y comanda   │  PID   │  (dron, Baxter  │
│                     │                 │  al robot)          │        │  o Atlas)       │
└───────────────────┘                 └─────────────────────┘        └────────────────┘
```

- **Parte a (drones):** el mensaje serial es un evento discreto (`WP,<punto>,x,y,z`) que
  dispara el avance al siguiente punto de ruta; el movimiento fluido lo genera el
  controlador **PID** de `gym-pybullet-drones` al perseguir cada punto.
- **Partes b y c (Baxter / Atlas):** el mensaje serial es una posición continua
  (`POS,x,y,z,grip`) actualizada ~20 veces por segundo; el movimiento fluido lo genera
  recalcular la **cinemática inversa (IK)** del brazo en cada ciclo hacia esa posición.
  La parte c reutiliza el mismo hardware de la parte b y extiende el mensaje para controlar
  dos manos y el cuerpo (ver el README de `parte-c-atlas-esp32/` para el detalle de por
  qué Atlas necesita un tratamiento distinto al ser un robot de muchos grados de libertad).

## Análisis general

La arquitectura separa claramente **decisión** (ESP32: qué debe hacer el robot) de
**ejecución física** (PyBullet: cómo llegar ahí, con PID o IK según el caso). Esto
permite:

- Probar el control del robot sin depender de hardware real, minimizando riesgo y costo.
- Reutilizar el mismo protocolo serial de texto plano en los tres escenarios, cambiando
  solo el contenido del mensaje según lo que necesita cada robot (waypoints discretos
  para el dron, posición continua para los brazos).
- Demostrar que el diseño de la consola (parte b) es independiente del robot concreto:
  la parte c reutiliza la misma ESP32 y el mismo protocolo sin ningún cambio de hardware,
  solo adaptando la lógica de control en el PC al nuevo robot.

## Paso a paso (resumen)

Ver el detalle completo en el README de cada parte. En general:

1. Crear el entorno de Python (`conda create -n drones python=3.12`) e instalar
   `pybullet`, `pyserial` y el repositorio correspondiente (`gym-pybullet-drones` para la
   parte a, `pybullet_robots` para las partes b y c).
2. Cargar el sketch de Arduino en la ESP32 (uno para drones, otro para la consola de 3
   pulsadores que comparten las partes b y c).
3. Conectar el hardware (botón BOOT para drones; 3 pulsadores para Baxter/Atlas).
4. Ejecutar el script de Python correspondiente con la ESP32 conectada y el Monitor
   Serie cerrado.

## Evidencia

Los videos de las partes a y b están en la carpeta `evidencias/` de cada subcarpeta. Los
de la parte c están en Google Drive:

- [Evidencia 1c](https://drive.google.com/file/d/1vqwhFFrqrlTJBy4VgVOPjl4dG4JZd5BO/view?usp=sharing)
- [Evidencia 2c](https://drive.google.com/file/d/18z95IucYVXfa1tptN7XGoLHjXvNHldaT/view?usp=sharing)

## Repositorios de referencia

- https://github.com/utiasDSL/gym-pybullet-drones
- https://github.com/erwincoumans/pybullet_robots
- https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py
- https://github.com/erwincoumans/pybullet_robots/blob/master/atlas.py
