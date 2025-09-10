from controller import Robot

# --- Constantes ---
MAX_SPEED = 5.24
CRUISING_SPEED = 4.0
RANGE_THRESHOLD = 1.2  # distancia mínima (m) para considerar obstáculo
SLOWDOWN_FACTOR = 0.5

# --- Inicialización ---
robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Motores
left_wheel = robot.getDevice("left wheel")
right_wheel = robot.getDevice("right wheel")
left_wheel.setPosition(float("inf"))
right_wheel.setPosition(float("inf"))
left_wheel.setVelocity(0.0)
right_wheel.setVelocity(0.0)

# Kinect Range Finder
kinect_range = robot.getDevice("kinect range")
kinect_range.enable(time_step)

width = kinect_range.getWidth()
height = kinect_range.getHeight()
half_width = width // 2
mid_height = height // 2

# --- Bucle principal ---
while robot.step(time_step) != -1:
    # Obtener valores del rango (distancias en metros)
    depth_image = kinect_range.getRangeImage()

    left_sum = 0.0
    right_sum = 0.0

    # Revisamos cada columna de la mitad izquierda y derecha
    for i in range(half_width):
        left_val = kinect_range.rangeImageGetDepth(depth_image, width, i, mid_height)
        right_val = kinect_range.rangeImageGetDepth(depth_image, width, width - i - 1, mid_height)

        if left_val < RANGE_THRESHOLD:
            left_sum += left_val
        if right_val < RANGE_THRESHOLD:
            right_sum += right_val

    # Velocidades base
    left_speed = CRUISING_SPEED
    right_speed = CRUISING_SPEED

    # Lógica: si hay más obstáculo en un lado → giramos hacia el otro
    if left_sum > right_sum and left_sum > 0:
        left_speed = SLOWDOWN_FACTOR * CRUISING_SPEED
        right_speed = CRUISING_SPEED
    elif right_sum > left_sum and right_sum > 0:
        left_speed = CRUISING_SPEED
        right_speed = SLOWDOWN_FACTOR * CRUISING_SPEED

    # Aplicar velocidades
    left_wheel.setVelocity(left_speed)
    right_wheel.setVelocity(right_speed)
