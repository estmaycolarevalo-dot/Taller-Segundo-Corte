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
> robot Baxter que permita una movilidad real.
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
| [`parte-b-c-baxter-esp32/`](./parte-b-c-baxter-esp32) | b) y c) Consola de mandos Baxter | Consola con 3 pulsadores que mueve el brazo Baxter en tiempo real (cinemática inversa) y controla la pinza para coger y mover un objeto. Esta misma solución cubre tanto el punto b (con agarre de objeto) como el punto c (movimiento fluido), ya que b es un superconjunto de los requisitos de c. |

Cada subcarpeta tiene su propio `README.md` con la arquitectura, el análisis y el paso a
paso detallado de esa parte.

## Arquitectura general

Los tres puntos siguen el mismo patrón: **ESP32 (entrada/decisión) → Serial USB
(comunicación) → PyBullet (ejecución física)**.

```
        ESP32                          PC (Python)                    PyBullet
┌───────────────────┐    Serial USB   ┌─────────────────────┐        ┌───────────────┐
│  Pulsadores /      │ ───────────────▶│  Script de control  │ ──────▶│  Robot         │
│  botón BOOT        │  texto plano    │  (interpreta el     │  IK /  │  simulado      │
│  (decide / mueve)  │                 │  mensaje y comanda   │  PID   │  (dron o       │
│                     │                 │  al robot)          │        │  brazo Baxter) │
└───────────────────┘                 └─────────────────────┘        └───────────────┘
```

- **Parte a (drones):** el mensaje serial es un evento discreto (`WP,<punto>,x,y,z`) que
  dispara el avance al siguiente punto de ruta; el movimiento fluido lo genera el
  controlador **PID** de `gym-pybullet-drones` al perseguir cada punto.
- **Partes b y c (Baxter):** el mensaje serial es una posición continua
  (`POS,x,y,z,grip`) actualizada ~20 veces por segundo; el movimiento fluido lo genera
  recalcular la **cinemática inversa (IK)** del brazo en cada ciclo hacia esa posición.

## Análisis general

La arquitectura separa claramente **decisión** (ESP32: qué debe hacer el robot) de
**ejecución física** (PyBullet: cómo llegar ahí, con PID o IK según el caso). Esto
permite:

- Probar el control del robot sin depender de hardware real, minimizando riesgo y costo.
- Reutilizar el mismo protocolo serial de texto plano en ambos escenarios, cambiando solo
  el contenido del mensaje según lo que necesita cada robot (waypoints discretos para el
  dron, posición continua para el brazo).
- Que el punto c quede automáticamente resuelto por la misma solución del punto b: si el
  brazo ya se mueve con fluidez real y controla la pinza (b), también cumple con
  "movimiento fluido... que permita una movilidad real" (c), sin duplicar código.

## Paso a paso (resumen)

Ver el detalle completo en el README de cada parte. En general:

1. Crear el entorno de Python (`conda create -n drones python=3.12`) e instalar
   `pybullet`, `pyserial` y el repositorio correspondiente (`gym-pybullet-drones` para la
   parte a, `pybullet_robots` para las partes b y c).
2. Cargar el sketch de Arduino en la ESP32 (uno distinto para drones y para Baxter).
3. Conectar el hardware (botón BOOT para drones; 3 pulsadores para Baxter).
4. Ejecutar el script de Python correspondiente con la ESP32 conectada y el Monitor
   Serie cerrado.

## Evidencia

Los videos de funcionamiento de cada parte van en la carpeta `evidencias/` de cada
subcarpeta (`parte-a-drones-esp32/evidencias/`, `parte-b-c-baxter-esp32/evidencias/`).

## Repositorios de referencia

- https://github.com/utiasDSL/gym-pybullet-drones
- https://github.com/erwincoumans/pybullet_robots
- https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py
