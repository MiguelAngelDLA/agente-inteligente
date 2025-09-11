from controller import Robot

# --- Configuración ---
TIME_STEP = 64
WHEEL_RADIUS = 0.15   # radio de las ruedas en metros (ajusta según tu robot)
AXLE_LENGTH = 0.6    # distancia entre ruedas en metros (ajusta según tu robot)

robot = Robot()

# Motores
motor1 = robot.getDevice("motor1")
motor2 = robot.getDevice("motor2")
motor1.setPosition(float("inf"))
motor2.setPosition(float("inf"))
motor1.setVelocity(0.0)
motor2.setVelocity(0.0)

# Encoders
enc1 = robot.getDevice("motor1_sensor")
enc2 = robot.getDevice("motor2_sensor")
enc1.enable(TIME_STEP)
enc2.enable(TIME_STEP)

# --- Estado odométrico ---
x, y, theta = 0.0, 0.0, 0.0
last_enc1 = enc1.getValue()
last_enc2 = enc2.getValue()

def update_odometry():
    global x, y, theta, last_enc1, last_enc2

    # Lectura actual
    left = enc1.getValue()
    right = enc2.getValue()

    # Incremento de cada rueda
    dl = (left - last_enc1) * WHEEL_RADIUS
    dr = (right - last_enc2) * WHEEL_RADIUS
    last_enc1, last_enc2 = left, right

    # Distancia y cambio angular
    dc = (dl + dr) / 2.0
    dtheta = (dr - dl) / AXLE_LENGTH

    # Actualizar pose
    x += dc * cos(theta + dtheta / 2.0)
    y += dc * sin(theta + dtheta / 2.0)
    theta += dtheta

# --- Movimiento simple: cubrir 10x10 en cuadrícula ---
from math import sin, cos, pi

SPEED = 3.0
STEP_DIST = 1.0   # avanzar 1 metro antes de girar

def go_forward():
    motor1.setVelocity(SPEED)
    motor2.setVelocity(SPEED)

def stop():
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def turn_left():
    motor1.setVelocity(-SPEED)
    motor2.setVelocity(SPEED)

def turn_right():
    motor1.setVelocity(SPEED)
    motor2.setVelocity(-SPEED)

# --- Bucle principal ---
import time
row = 0
direction = 1

while robot.step(TIME_STEP) != -1:
    update_odometry()

    # Ejemplo: movimiento en zig-zag para cubrir 10x10
    if row < 10:
        if (direction == 1 and x < (row+1)*STEP_DIST) or (direction == -1 and x > (row+1)*STEP_DIST - 10):
            go_forward()
        else:
            stop()
            time.sleep(0.5)
            if direction == 1:
                turn_left()
            else:
                turn_right()
            time.sleep(1.0)  # tiempo de giro (ajustar según robot)
            stop()
            row += 1
            direction *= -1
    else:
        stop()
