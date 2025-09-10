from controller import Robot, Motor

# ---------------- CONFIG ----------------
TIME_STEP = 64
MAX_SPEED = 6.28
NUM_GNOMES = 10
GNOME_PREFIX = "GNOME"

# ---------------- INIT ----------------
robot = Robot()

# Motores
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for m in [left_motor, right_motor]:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

# Sensores de proximidad (los que me diste)
sensor_names = [
    "front infrared sensor",
    "front left infrared sensor",
    "front left ultrasonic sensor",
    "front right infrared sensor",
    "front right ultrasonic sensor",
    "front ultrasonic sensor",
    "ground front left infrared sensor",
    "ground front right infrared sensor",
    "ground left infrared sensor",
    "ground right infrared sensor",
    "left infrared sensor",
    "left ultrasonic sensor",
    "rear infrared sensor",
    "rear left infrared sensor",
    "rear right infrared sensor",
    "right infrared sensor",
    "right ultrasonic sensor"
]

sensors = {}
for name in sensor_names:
    s = robot.getDevice(name)
    s.enable(TIME_STEP)
    sensors[name] = s

# Cámara
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

# Lista de gnomos
gnomes = [robot.getFromDef(GNOME_PREFIX if i == 0 else f"{GNOME_PREFIX}({i})") for i in range(NUM_GNOMES)]
gnome_active = [True] * NUM_GNOMES
gnomes_collected = 0

# ---------------- MAIN LOOP ----------------
while robot.step(TIME_STEP) != -1:
    objects = camera.getRecognitionObjects()
    gnome_detected = None

    # Buscar gnomos visibles
    for obj in objects:
        if obj.getModel().startswith(GNOME_PREFIX):
            gnome_detected = obj
            break

    if gnome_detected:
        # Posición del gnomo en la imagen
        image_width = camera.getWidth()
        center_x = image_width / 2
        gnome_x = gnome_detected.getPositionOnImage()[0]

        # Seguir al gnomo (alineación simple)
        if gnome_x < center_x - 10:  # gnomo a la izquierda
            left_motor.setVelocity(0.3 * MAX_SPEED)
            right_motor.setVelocity(0.6 * MAX_SPEED)
        elif gnome_x > center_x + 10:  # gnomo a la derecha
            left_motor.setVelocity(0.6 * MAX_SPEED)
            right_motor.setVelocity(0.3 * MAX_SPEED)
        else:  # centrado → avanzar
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)

        # Comprobar contacto con sensores frontales
        front_ir = sensors["front infrared sensor"].getValue()
        front_ultra = sensors["front ultrasonic sensor"].getValue()

        if front_ir > 300 or front_ultra > 300:  # umbral de contacto
            gnome_name = gnome_detected.getModel()
            print(f"✅ Gnomo tocado: {gnome_name}")
            node = robot.getFromDef(gnome_name)
            if node:
                node.getField("translation").setSFVec3f([0, -10, 0])  # desaparecer
                gnomes_collected += 1
                # marcar como inactivo
                for idx, g in enumerate(gnomes):
                    if g and g.getDef() == gnome_name:
                        gnome_active[idx] = False
                        break

    else:
        # No hay gnomos visibles → patrullar girando
        left_motor.setVelocity(0.5 * MAX_SPEED)
        right_motor.setVelocity(-0.5 * MAX_SPEED)

    # Si ya recolectó todos, detenerse
    if gnomes_collected == NUM_GNOMES:
        print("🎉 Todos los gnomos recolectados!")
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
