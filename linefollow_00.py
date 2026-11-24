#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import Font

ev3 = EV3Brick()
try:
    ev3.screen.set_font(Font(size=14, bold=True))
except Exception:
    pass

motor_esq = Motor(Port.A)
motor_dir = Motor(Port.B)
sensor_esq = ColorSensor(Port.S2)
sensor_dir = ColorSensor(Port.S1)

V_BASE = 90
KP = 2.2
KD = 0.5
KI = 0
PRETO = 25
CRUZAMENTO_MS = 350


def desenhar_tela(modo):
    ev3.screen.clear()
    ev3.screen.draw_text(4, 4, "=== PID EV3 ===")
    s_kp = ">" if modo == 0 else " "
    s_kd = ">" if modo == 0 else " "
    s_vb = ">" if modo == 1 else " "
    ev3.screen.draw_text(4, 26, "{}KP: {:.2f} (Cim/Bai)".format(s_kp, KP))
    ev3.screen.draw_text(4, 46, "{}KD: {:.2f} (Dir/Esq)".format(s_kd, KD))
    ev3.screen.draw_text(4, 66, "{}VEL: {}%".format(s_vb, V_BASE))
    if modo == 0:
        ev3.screen.draw_text(4, 90, "[MEIO] Largar/Parar")
        ev3.screen.draw_text(4, 108, "[SEG.MEIO] Mudar VEL")
    else:
        ev3.screen.draw_text(4, 90, "Cim/Bai: VEL +-5")
        ev3.screen.draw_text(4, 108, "[MEIO] Voltar PID")


def esperar_soltar():
    while ev3.buttons.pressed():
        wait(20)
    wait(50)


def seguir_linha():
    ler_esq = sensor_esq.reflection
    ler_dir = sensor_dir.reflection
    dc_esq = motor_esq.dc
    dc_dir = motor_dir.dc
    botoes = ev3.buttons.pressed
    centro = Button.CENTER
    relogio = StopWatch()
    tempo = relogio.time

    v_base = V_BASE
    kp = KP
    kd = KD
    ki = KI
    preto = PRETO
    cruz_ms = CRUZAMENTO_MS

    erro_ant = ler_esq() - ler_dir()
    integral = 0.0
    deriv = 0.0
    em_cruz = False
    t_cruz = 0
    ult_esq = 999
    ult_dir = 999
    loops = 0
    t_prox = tempo()

    try:
        while True:
            now = tempo()
            if now < t_prox:
                if t_prox - now > 1:
                    wait(1)
                continue
            t_prox = now + 3

            if (loops & 31) == 0 and centro in botoes():
                break

            le = ler_esq()
            ld = ler_dir()

            if le <= preto and ld <= preto:
                em_cruz = True
                t_cruz = now
                erro = 0
                deriv = 0.0
                integral = 0.0
                erro_ant = 0
            elif em_cruz:
                if (le > preto and ld > preto) or (now - t_cruz > cruz_ms):
                    em_cruz = False
                    erro = le - ld
                    deriv = 0.0
                    integral = 0.0
                    erro_ant = erro
                else:
                    erro = 0
                    deriv = 0.0
                    integral = 0.0
                    erro_ant = 0
            else:
                erro = le - ld
                deriv = erro - erro_ant
                erro_ant = erro
                if ki != 0.0:
                    integral += erro
                    if integral > 100.0:
                        integral = 100.0
                    elif integral < -100.0:
                        integral = -100.0

            corr = kp * erro + kd * deriv + ki * integral
            esq = v_base + corr
            dir_v = v_base - corr

            if esq > 100.0:
                dir_v -= esq - 100.0
                esq = 100.0
            elif dir_v > 100.0:
                esq -= dir_v - 100.0
                dir_v = 100.0

            if esq > 100.0:
                esq = 100.0
            elif esq < -100.0:
                esq = -100.0
            if dir_v > 100.0:
                dir_v = 100.0
            elif dir_v < -100.0:
                dir_v = -100.0

            esq_i = int(esq)
            dir_i = int(dir_v)

            if esq_i != ult_esq:
                dc_esq(esq_i)
                ult_esq = esq_i
            if dir_i != ult_dir:
                dc_dir(dir_i)
                ult_dir = dir_i

            loops += 1
    finally:
        motor_esq.brake()
        motor_dir.brake()


modo = 0
desenhar_tela(modo)
try:
    while True:
        botoes = ev3.buttons.pressed()
        if modo == 0:
            if Button.UP in botoes:
                KP = round(KP + 0.05, 2)
                ev3.speaker.beep(1000, 30)
                desenhar_tela(modo)
                wait(180)
            elif Button.DOWN in botoes:
                KP = max(0.0, round(KP - 0.05, 2))
                ev3.speaker.beep(700, 30)
                desenhar_tela(modo)
                wait(180)
            elif Button.RIGHT in botoes:
                KD = round(KD + 0.05, 2)
                ev3.speaker.beep(1100, 30)
                desenhar_tela(modo)
                wait(180)
            elif Button.LEFT in botoes:
                KD = max(0.0, round(KD - 0.05, 2))
                ev3.speaker.beep(600, 30)
                desenhar_tela(modo)
                wait(180)
            elif Button.CENTER in botoes:
                t_hold = StopWatch()
                while Button.CENTER in ev3.buttons.pressed():
                    if t_hold.time() >= 500:
                        modo = 1
                        ev3.speaker.beep(900, 60)
                        desenhar_tela(modo)
                        esperar_soltar()
                        break
                    wait(20)
                if modo == 0:
                    ev3.speaker.beep(1400, 60)
                    wait(300)
                    seguir_linha()
                    ev3.speaker.beep(500, 80)
                    esperar_soltar()
                    desenhar_tela(modo)
        else:
            if Button.UP in botoes:
                V_BASE = min(90, V_BASE + 5)
                ev3.speaker.beep(1200, 30)
                desenhar_tela(modo)
                wait(180)
            elif Button.DOWN in botoes:
                V_BASE = max(15, V_BASE - 5)
                ev3.speaker.beep(600, 30)
                desenhar_tela(modo)
                wait(180)
            elif Button.CENTER in botoes:
                esperar_soltar()
                modo = 0
                ev3.speaker.beep(900, 40)
                desenhar_tela(modo)
        wait(20)
finally:
    motor_esq.brake()
    motor_dir.brake()
