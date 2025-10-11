import threading

# --- CONFIGURACIÓN PRINCIPAL ---
CONFIG = {
    "GRID_WIDTH": 10, "GRID_HEIGHT": 10,
    "num_gnomes": 10, "num_obstacles": 15, "animation_speed": 4.0,
    "battery_capacity": 150.0, "battery_low_threshold": 30.0,
    "drain_base": 0.05, "drain_move": 2.0, "drain_turn": 1.0, "charge_rate": 10.0,
    "num_slow_cells": 20, "cost_slow_cell": 8, "drain_slow_modifier": 3.0
}

# --- CONSTANTES DE LA SIMULACIÓN ---
CELL_SIZE = 60
UI_WIDTH = 300
WEB_SERVER_PORT = 8000

# --- CONSTANTES DE MEMORIA ---
MEM_UNKNOWN = -1
MEM_EMPTY = 0
MEM_OBSTACLE = 1
MEM_GNOME = 2
MEM_HOME = 3
MEM_SLOW = 4

# --- COLORES ---
COLOR_BG = (20, 20, 30)
COLOR_GRID = (40, 40, 50)
COLOR_OBSTACLE = (100, 100, 110)
COLOR_GNOME = (220, 50, 50)
COLOR_HOME = (200, 200, 200, 100)
COLOR_ROBOT = (50, 150, 250)
COLOR_PATH = (50, 150, 250, 60)
COLOR_TEXT = (240, 240, 240)
COLOR_TEXT_HIGHLIGHT = (100, 255, 100)
COLOR_BATTERY_GREEN = (50, 200, 50)
COLOR_BATTERY_ORANGE = (250, 160, 50)
COLOR_BATTERY_RED = (220, 50, 50)
COLOR_VISION = (50, 150, 250, 50)
COLOR_SLOW_TERRAIN = (139, 69, 19)

# --- DATOS COMPARTIDOS PARA EL SERVIDOR WEB ---
shared_data = {
    "memory_grid": [],
    "robot_pos": [0, 0],
    "grid_width": CONFIG['GRID_WIDTH'],
    "grid_height": CONFIG['GRID_HEIGHT']
}
data_lock = threading.Lock()