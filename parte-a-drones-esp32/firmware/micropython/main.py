"""
ESP32 (MicroPython) - control real-to-sim
Guardar como main.py en el dispositivo (Thonny: Archivo > Guardar como > Dispositivo MicroPython)
Envia por serial: WP,<nombre>,<x>,<y>,<z>
"""
from machine import Pin
import time

PUNTOS = [
    ("A", 0.0, 0.0, 1.0),
    ("B", 1.0, 1.0, 1.0),
    ("C", 2.0, 0.0, 1.0),
]
MODO = "boton"
INTERVALO_AUTO_S = 8

boton = Pin(0, Pin.IN, Pin.PULL_UP)
led = Pin(2, Pin.OUT)


def enviar(punto):
    nombre, x, y, z = punto
    print("WP,{},{},{},{}".format(nombre, x, y, z))
    led.value(1)
    time.sleep_ms(150)
    led.value(0)


time.sleep(2)
print("ESP32 listo. Modo:", MODO)

i = 0
while True:
    if MODO == "boton":
        if boton.value() == 0:
            enviar(PUNTOS[i])
            i = (i + 1) % len(PUNTOS)
            while boton.value() == 0:
                time.sleep_ms(20)
            time.sleep_ms(200)
    else:
        enviar(PUNTOS[i])
        i = (i + 1) % len(PUNTOS)
        time.sleep(INTERVALO_AUTO_S)
    time.sleep_ms(10)
