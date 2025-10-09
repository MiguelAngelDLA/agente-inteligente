import pygame
import math
import random
import heapq
import json
import threading
import http.server
import socketserver

# --- CONFIGURACIÓN INICIAL ---
# El tamaño del mundo ahora se define aquí y puede ser modificado por los sliders.
CONFIG = {
    "GRID_WIDTH": 10, "GRID_HEIGHT": 10,
    "num_gnomes": 10, "num_obstacles": 15, "animation_speed": 4.0,
    "battery_capacity": 150.0, "battery_low_threshold": 30.0,
    "drain_base": 0.05, "drain_move": 2.0, "drain_turn": 1.0, "charge_rate": 10.0,
    "num_slow_cells": 20, "cost_slow_cell": 8, "drain_slow_modifier": 3.0
}
WEB_SERVER_PORT = 8000
CELL_SIZE = 60 # Se mantiene constante para el dibujado en Pygame

# --- Constantes y Colores ---
MEM_UNKNOWN, MEM_EMPTY, MEM_OBSTACLE = -1, 0, 1
MEM_GNOME, MEM_HOME, MEM_SLOW = 2, 3, 4
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


# --- Datos Compartidos para el Servidor Web ---
shared_data = {
    "memory_grid": [],
    "robot_pos": [0, 0],
    "grid_width": CONFIG['GRID_WIDTH'],
    "grid_height": CONFIG['GRID_HEIGHT']
}
data_lock = threading.Lock()

# --- Clases de Lógica ---
class Node:
    def __init__(self, position, parent=None):
        self.position, self.parent = position, parent
        self.g, self.h, self.f = 0, 0, 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f
    def __repr__(self): return f"Node({self.position})"

class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, initial_val, label, config_key):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val, self.max_val, self.val = min_val, max_val, initial_val
        self.label, self.config_key = label, config_key
        self.grabbed = False
        self.update_config()

    def update_config(self):
        if isinstance(CONFIG[self.config_key], int):
            self.val = int(round(self.val))
        CONFIG[self.config_key] = self.val

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(event.pos):
            self.grabbed = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.grabbed = False
        elif event.type == pygame.MOUSEMOTION and self.grabbed:
            mouse_x = event.pos[0]
            new_val = (mouse_x - self.rect.x) / self.rect.width
            self.val = self.min_val + new_val * (self.max_val - self.min_val)
            self.val = max(self.min_val, min(self.max_val, self.val))
            self.update_config()

    def draw(self, screen, font):
        label_surf = font.render(f"{self.label}: {self.val:.1f}", True, COLOR_TEXT)
        screen.blit(label_surf, (self.rect.x, self.rect.y - 22))
        pygame.draw.rect(screen, (30, 30, 40), self.rect)
        pygame.draw.rect(screen, (80, 80, 90), self.rect, 1)
        handle_x = self.rect.x + (self.val - self.min_val) / (self.max_val - self.min_val) * self.rect.width
        pygame.draw.circle(screen, COLOR_ROBOT, (int(handle_x), self.rect.centery), 8)

def find_path_astar(memory_grid, start, end):
    start_node, end_node = Node(start), Node(end)
    open_list, closed_list = [], set()
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]
        
        (r, c) = current_node.position
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor_pos = (r + dr, c + dc)
            if not (0 <= neighbor_pos[0] < CONFIG['GRID_HEIGHT'] and 0 <= neighbor_pos[1] < CONFIG['GRID_WIDTH']) or \
               memory_grid[neighbor_pos[0]][neighbor_pos[1]] in [MEM_OBSTACLE, MEM_UNKNOWN] or \
               neighbor_pos in closed_list:
                continue
            
            cost = CONFIG["cost_slow_cell"] if memory_grid[neighbor_pos[0]][neighbor_pos[1]] == MEM_SLOW else 1
            neighbor_node = Node(neighbor_pos, current_node)
            neighbor_node.g = current_node.g + cost
            neighbor_node.h = abs(neighbor_pos[0] - end_node.position[0]) + abs(neighbor_pos[1] - end_node.position[1])
            neighbor_node.f = neighbor_node.g + neighbor_node.h
            if any(n for n in open_list if neighbor_node == n and neighbor_node.g >= n.g):
                continue
            heapq.heappush(open_list, neighbor_node)
    return None

def bresenham_line(start, end):
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    points = []
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return points

class World:
    def __init__(self):
        self.width, self.height = CONFIG['GRID_WIDTH'], CONFIG['GRID_HEIGHT']
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.gnomes = []
        self.home_base = (0, 0)
        if self.home_base[0] < self.height and self.home_base[1] < self.width:
            self.grid[self.home_base[0]][self.home_base[1]] = 3
        self.generate_world_elements()

    def generate_world_elements(self):
        self.grid = [[0 if (r, c) != self.home_base else 3 for c in range(self.width)] for r in range(self.height)]
        self.gnomes = []
        
        for _ in range(CONFIG["num_obstacles"]):
            while True:
                r, c = random.randint(0, self.height - 1), random.randint(0, self.width - 1)
                if self.grid[r][c] == 0 and math.dist((r, c), self.home_base) > 2:
                    self.grid[r][c] = 1
                    break
        for _ in range(CONFIG["num_slow_cells"]):
            while True:
                r, c = random.randint(0, self.height - 1), random.randint(0, self.width - 1)
                if self.grid[r][c] == 0:
                    self.grid[r][c] = 4
                    break
        for _ in range(CONFIG["num_gnomes"]):
            while True:
                r, c = random.randint(0, self.height - 1), random.randint(0, self.width - 1)
                if self.grid[r][c] == 0:
                    self.grid[r][c] = 2
                    self.gnomes.append((r, c))
                    break

class SimulatedRobot:
    def __init__(self):
        self.row, self.col, self.theta = 0, 0, 0.0
        self.action, self.path, self.state = 'ESPERANDO', [], 'BUSCANDO'
        self.animation_progress = 0.0
        self.battery, self.inventory = CONFIG["battery_capacity"], []
        self.discovered_gnomes, self.potential_decisions = set(), []
        self.patrol_points, self.patrol_index = self.generate_patrol_points(), 0
        self.current_target_pos, self.saved_state, self.saved_target_pos = None, None, None
        self.target_theta = 0.0
        self.view_pattern = self.generate_view_pattern()
        self.memory_grid = [[MEM_UNKNOWN for _ in range(CONFIG['GRID_WIDTH'])] for _ in range(CONFIG['GRID_HEIGHT'])]

    def generate_view_pattern(self):
        base_pattern = set()
        max_dist = 4
        for r in range(-max_dist, max_dist + 1):
            for c in range(1, max_dist + 1):
                if abs(r) <= c:
                    base_pattern.add((r, c))
        return {
            0: list(base_pattern),
            math.pi/2: [(-c, r) for r, c in base_pattern],
            math.pi: [(-r, -c) for r, c in base_pattern],
            -math.pi/2: [(c, -r) for r, c in base_pattern]
        }

    def generate_patrol_points(self):
        points = []
        for r in range(CONFIG['GRID_HEIGHT']):
            row_points = range(CONFIG['GRID_WIDTH']) if r % 2 == 0 else range(CONFIG['GRID_WIDTH'] - 1, -1, -1)
            for c in row_points:
                points.append((r, c))
        return points

    def update_memory_with_los(self, world):
        self.memory_grid[self.row][self.col] = world.grid[self.row][self.col]
        closest_theta_key = min(self.view_pattern.keys(), key=lambda k: abs(k - self.theta))
        
        for dr, dc in self.view_pattern.get(closest_theta_key, []):
            target_r, target_c = self.row + dr, self.col + dc
            if not (0 <= target_r < CONFIG['GRID_HEIGHT'] and 0 <= target_c < CONFIG['GRID_WIDTH']):
                continue
            
            line = bresenham_line((self.row, self.col), (target_r, target_c))
            for r, c in line:
                self.memory_grid[r][c] = world.grid[r][c]
                if self.memory_grid[r][c] == MEM_OBSTACLE and (r, c) != (self.row, self.col):
                    break

    def scan_for_gnomes(self, world):
        visible_gnomes = set()
        closest_theta_key = min(self.view_pattern.keys(), key=lambda k: abs(k - self.theta))
        for dr, dc in self.view_pattern.get(closest_theta_key, []):
            check_r, check_c = self.row + dr, self.col + dc
            if 0 <= check_r < CONFIG['GRID_HEIGHT'] and 0 <= check_c < CONFIG['GRID_WIDTH']:
                if world.grid[check_r][check_c] == 2 and (check_r, check_c) not in self.discovered_gnomes:
                    visible_gnomes.add((check_r, check_c))
        return list(visible_gnomes)

    def update(self, world, dt):
        self.update_memory_with_los(world)
        self.update_logic(world, dt)
        self.execute_movement(world, dt)
        with data_lock:
            shared_data["memory_grid"] = self.memory_grid
            shared_data["robot_pos"] = [self.row, self.col]
            shared_data["grid_width"] = CONFIG['GRID_WIDTH']
            shared_data["grid_height"] = CONFIG['GRID_HEIGHT']

    def execute_movement(self, world, dt):
        if self.action == 'ESPERANDO': return
        drain = CONFIG["drain_base"] * dt
        if self.action == 'GIRANDO': drain += CONFIG["drain_turn"] * dt
        elif self.action == 'MOVIENDOSE': drain += CONFIG["drain_move"] * dt
        if world.grid[self.row][self.col] == 4: drain *= CONFIG["drain_slow_modifier"]
        self.battery = max(0, self.battery - drain)
        self.animation_progress += CONFIG["animation_speed"] * dt
        if self.animation_progress >= 1.0:
            self.animation_progress %= 1.0
            if self.action == 'GIRANDO': self.theta = self.target_theta
            elif self.action == 'MOVIENDOSE' and self.path: self.row, self.col = self.path.pop(0)
            if not self.path: self.action = 'ESPERANDO'
            else: self.start_next_path_step()

    def start_next_path_step(self):
        if not self.path:
            self.action = 'ESPERANDO'
            return
        next_pos = self.path[0]
        dr, dc = next_pos[0] - self.row, next_pos[1] - self.col
        self.target_theta = math.atan2(-dr, dc)
        angle_diff = (self.target_theta - self.theta + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_diff) > 0.1: self.action = 'GIRANDO'
        else: self.action = 'MOVIENDOSE'

    def update_logic(self, world, dt):
        if self.battery <= CONFIG["battery_low_threshold"] and self.state not in ['VOLVIENDO_A_CASA', 'CARGANDO']:
            self.saved_state, self.saved_target_pos = self.state, self.current_target_pos
            self.current_target_pos = world.home_base
            self.path = find_path_astar(self.memory_grid, (self.row, self.col), self.current_target_pos)
            self.state = 'VOLVIENDO_A_CASA'
            if self.path: self.start_next_path_step()
            return

        if self.state == 'BUSCANDO':
            self.potential_decisions = []
            found_gnomes = self.scan_for_gnomes(world)
            if found_gnomes:
                for gnome_pos in found_gnomes:
                    path = find_path_astar(self.memory_grid, (self.row, self.col), gnome_pos)
                    if path and len(path) > 1:
                        cost = len(path) - 1
                        benefit = 1000 / cost
                        self.potential_decisions.append({"target": gnome_pos, "path": path, "cost": cost, "benefit": benefit})
                if self.potential_decisions:
                    best_decision = max(self.potential_decisions, key=lambda d: d['benefit'])
                    self.state = 'YENDO_AL_GNOMO'
                    self.current_target_pos = best_decision['target']
                    self.path = best_decision['path']
                    self.discovered_gnomes.add(best_decision['target'])
                    if self.path: self.start_next_path_step()
                    return
            
            if self.action == 'ESPERANDO':
                self.potential_decisions = []
                if self.patrol_index < len(self.patrol_points):
                    next_patrol = self.patrol_points[self.patrol_index]
                    if (self.row, self.col) != next_patrol:
                        self.path = find_path_astar(self.memory_grid, (self.row, self.col), next_patrol)
                        self.current_target_pos = next_patrol
                    self.patrol_index += 1
                else: self.state = 'TERMINADO'
        
        if self.action != 'ESPERANDO': return

        if self.state == 'YENDO_AL_GNOMO':
            gnome_pos = self.current_target_pos
            if gnome_pos in world.gnomes:
                world.gnomes.remove(gnome_pos)
                world.grid[gnome_pos[0]][gnome_pos[1]] = 0
                self.inventory.append(gnome_pos)
            self.state = 'YENDO_A_BASE'
            self.current_target_pos = world.home_base
            self.path = find_path_astar(self.memory_grid, (self.row, self.col), self.current_target_pos)
        elif self.state == 'YENDO_A_BASE':
            if self.inventory: self.inventory.pop(0)
            self.state = 'BUSCANDO'
            self.current_target_pos = None
        elif self.state == 'VOLVIENDO_A_CASA':
            self.state = 'CARGANDO'
        elif self.state == 'CARGANDO':
            self.battery += CONFIG["charge_rate"] * dt
            if self.battery >= CONFIG["battery_capacity"]:
                self.battery = CONFIG["battery_capacity"]
                self.state = self.saved_state if self.saved_state else 'BUSCANDO'
                self.current_target_pos = self.saved_target_pos
                if self.current_target_pos:
                    self.path = find_path_astar(self.memory_grid, (self.row, self.col), self.current_target_pos)
                else: self.state = 'BUSCANDO'; self.path = None
                self.saved_state, self.saved_target_pos = None, None
        
        if self.action == 'ESPERANDO' and self.path: self.start_next_path_step()

# --- Servidor Web ---
class RequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = '/index.html'
        if self.path == '/mapdata':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with data_lock:
                self.wfile.write(json.dumps(shared_data).encode('utf-8'))
        else:
            return http.server.SimpleHTTPRequestHandler.do_GET(self)

def start_web_server():
    with socketserver.TCPServer(("", WEB_SERVER_PORT), RequestHandler) as httpd:
        print(f"Servidor web iniciado en http://localhost:{WEB_SERVER_PORT}")
        httpd.serve_forever()

# --- Funciones de dibujado ---
def draw_world(screen, world):
    for r in range(world.height):
        for c in range(world.width):
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            cell_type = world.grid[r][c]
            if cell_type == 1: pygame.draw.rect(screen, COLOR_OBSTACLE, rect)
            elif cell_type == 2: pygame.draw.circle(screen, COLOR_GNOME, rect.center, int(CELL_SIZE * 0.3))
            elif cell_type == 3: pygame.draw.rect(screen, COLOR_HOME, rect, border_radius=5)
            elif cell_type == 4: pygame.draw.rect(screen, COLOR_SLOW_TERRAIN, rect)

def draw_grid(screen, width, height):
    for x in range(0, width, CELL_SIZE): pygame.draw.line(screen, COLOR_GRID, (x, 0), (x, height))
    for y in range(0, height, CELL_SIZE): pygame.draw.line(screen, COLOR_GRID, (0, y), (width, y))

def draw_path(screen, robot):
    if robot.path:
        for pos in robot.path:
            rect = pygame.Rect(pos[1] * CELL_SIZE, pos[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, COLOR_PATH, rect)

def draw_vision_cone(screen, robot):
    vision_surface = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    closest_theta_key = min(robot.view_pattern.keys(), key=lambda k: abs(k - robot.theta))
    for dr, dc in robot.view_pattern.get(closest_theta_key, []):
        r, c = robot.row + dr, robot.col + dc
        if 0 <= r < CONFIG['GRID_HEIGHT'] and 0 <= c < CONFIG['GRID_WIDTH']:
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(vision_surface, COLOR_VISION, rect)
    screen.blit(vision_surface, (0, 0))

def draw_robot(screen, robot):
    start_px = (robot.col * CELL_SIZE + CELL_SIZE/2, robot.row * CELL_SIZE + CELL_SIZE/2)
    end_px = start_px
    if robot.action == 'MOVIENDOSE' and robot.path:
        next_r, next_c = robot.path[0]
        end_px = (next_c * CELL_SIZE + CELL_SIZE/2, next_r * CELL_SIZE + CELL_SIZE/2)
    t = robot.animation_progress
    pos_x = start_px[0] * (1-t) + end_px[0] * t
    pos_y = start_px[1] * (1-t) + end_px[1] * t
    target_theta_interp = robot.target_theta if robot.action == 'GIRANDO' else robot.theta
    angle_diff = (target_theta_interp - robot.theta + math.pi) % (2 * math.pi) - math.pi
    interp_angle = robot.theta + angle_diff * t
    pygame.draw.circle(screen, COLOR_ROBOT, (pos_x, pos_y), int(CELL_SIZE * 0.4))
    line_end_x = pos_x + math.cos(interp_angle) * (CELL_SIZE * 0.35)
    line_end_y = pos_y - math.sin(interp_angle) * (CELL_SIZE * 0.35)
    pygame.draw.line(screen, COLOR_TEXT, (pos_x, pos_y), (line_end_x, line_end_y), 3)

def draw_ui(screen, robot, font, small_font, sliders, applied_w, applied_h):
    ui_width = 300
    screen_width = applied_w * CELL_SIZE
    ui_rect = pygame.Rect(screen_width, 0, ui_width, screen.get_height())
    pygame.draw.rect(screen, (50, 50, 60), ui_rect)
    y = 20
    def draw_text(text, font_obj, pos, color=COLOR_TEXT):
        text_surf = font_obj.render(text, True, color)
        screen.blit(text_surf, pos)
        return text_surf.get_height()

    y += draw_text("ESTADO DEL ROBOT", font, (screen_width + 20, y)) + 5
    y += draw_text(f"Estado: {robot.state} | Acción: {robot.action}", small_font, (screen_width + 25, y)) + 15
    y += draw_text("Batería", font, (screen_width + 20, y)) + 5
    pct = robot.battery / CONFIG["battery_capacity"] if CONFIG["battery_capacity"] > 0 else 0
    bar_w, bar_h = ui_width - 40, 15
    bar_color = COLOR_BATTERY_RED if pct < (CONFIG["battery_low_threshold"]/CONFIG["battery_capacity"]) else COLOR_BATTERY_ORANGE if pct < 0.5 else COLOR_BATTERY_GREEN
    pygame.draw.rect(screen, (20,20,30), (screen_width + 20, y, bar_w, bar_h))
    pygame.draw.rect(screen, bar_color, (screen_width + 20, y, bar_w * pct, bar_h))
    y += bar_h + 15

    y += draw_text("ANÁLISIS DE DECISIÓN", font, (screen_width + 20, y)) + 5
    if robot.potential_decisions:
        sorted_decisions = sorted(robot.potential_decisions, key=lambda d: d['benefit'], reverse=True)
        for i, decision in enumerate(sorted_decisions):
            is_best = (decision['target'] == robot.current_target_pos)
            color = COLOR_TEXT_HIGHLIGHT if is_best else COLOR_TEXT
            draw_text(f"Op: Ir a {decision['target']} (Costo: {decision['cost']})", small_font, (screen_width + 25, y), color)
            y += 18
            if y > 450: break
    else:
        y += draw_text("Patrullando o sin opciones...", small_font, (screen_width + 25, y))
    
    y = 480
    if applied_w != CONFIG['GRID_WIDTH'] or applied_h != CONFIG['GRID_HEIGHT']:
        alert_text = font.render("Presiona 'R' para aplicar cambios", True, (255, 200, 0))
        text_rect = alert_text.get_rect(center=(screen_width + ui_width / 2, y))
        screen.blit(alert_text, text_rect)
    y += 30

    for slider in sliders:
        slider.draw(screen, small_font)
        y += 40

# --- Clase Principal del Juego ---
class Game:
    def __init__(self):
        pygame.init()
        self.font = pygame.font.Font(None, 28)
        self.small_font = pygame.font.Font(None, 20)
        self.clock = pygame.time.Clock()
        self.simulation_running = True
        self.applied_grid_width = CONFIG['GRID_WIDTH']
        self.applied_grid_height = CONFIG['GRID_HEIGHT']
        self.setup_simulation()

    def setup_simulation(self):
        self.applied_grid_width = CONFIG['GRID_WIDTH']
        self.applied_grid_height = CONFIG['GRID_HEIGHT']

        screen_width = self.applied_grid_width * CELL_SIZE
        screen_height = self.applied_grid_height * CELL_SIZE
        total_width = screen_width + 300

        self.screen = pygame.display.set_mode((total_width, screen_height))
        pygame.display.set_caption("Simulación de Robot (Visor en Web)")

        self.world = World()
        self.robot = SimulatedRobot()
        
        self.sliders = [
            Slider(screen_width + 25, 520, 300 - 50, 12, 5, 30, CONFIG['GRID_WIDTH'], "Ancho Mundo", 'GRID_WIDTH'),
            Slider(screen_width + 25, 560, 300 - 50, 12, 5, 30, CONFIG['GRID_HEIGHT'], "Alto Mundo", 'GRID_HEIGHT'),
            Slider(screen_width + 25, 600, 300 - 50, 12, 1.0, 10.0, CONFIG['animation_speed'], "Velocidad Anim.", 'animation_speed'),
        ]

    def run(self):
        running = True
        while running:
            dt_sec = self.clock.tick(60) / 1000.0
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                for slider in self.sliders:
                    slider.handle_event(event)
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.simulation_running = not self.simulation_running
                    if event.key == pygame.K_r:
                        self.setup_simulation()

            if self.simulation_running:
                self.robot.update(self.world, dt_sec)
                if self.robot.state == 'TERMINADO' or self.robot.battery <= 0:
                    self.simulation_running = False

            screen_width = self.applied_grid_width * CELL_SIZE
            screen_height = self.applied_grid_height * CELL_SIZE
            
            self.screen.fill(COLOR_BG)
            draw_world(self.screen, self.world)
            draw_grid(self.screen, screen_width, screen_height)
            draw_path(self.screen, self.robot)
            draw_vision_cone(self.screen, self.robot)
            draw_robot(self.screen, self.robot)
            draw_ui(self.screen, self.robot, self.font, self.small_font, self.sliders, self.applied_grid_width, self.applied_grid_height)

            pygame.display.flip()
        
        pygame.quit()

# --- Punto de Entrada ---
if __name__ == '__main__':
    server_thread = threading.Thread(target=start_web_server, daemon=True)
    server_thread.start()
    
    game = Game()
    game.run()