#!/usr/bin/env pybricks-micropython
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

motor_esq = Motor(Port.A)
motor_dir = Motor(Port.B)
sensor_esq = ColorSensor(Port.S2)
sensor_dir = ColorSensor(Port.S1)

V_BASE = 50
KP = 2.2
KD = 0.5
KI = 0.01

erro_anterior, integral = 0, 0
while True:
    erro = sensor_esq.reflection() - sensor_dir.reflection()
    integral += erro
    derivativo = erro - erro_anterior
    correcao = (KP * erro) + (KD * derivativo) + (KI * integral)
    motor_esq.dc(V_BASE + correcao)
    motor_dir.dc(V_BASE - correcao)
    erro_anterior = erro
    wait(10)