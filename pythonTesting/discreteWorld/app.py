import pygame
import math
import random
import heapq

# --- Configuración de Pygame y Pantalla ---
CELL_SIZE = 60
GRID_WIDTH, GRID_HEIGHT = 10, 10
UI_WIDTH = 300
SCREEN_WIDTH = GRID_WIDTH * CELL_SIZE
SCREEN_HEIGHT = GRID_HEIGHT * CELL_SIZE
TOTAL_WIDTH = SCREEN_WIDTH + UI_WIDTH

# Colores
COLOR_BG = (20, 20, 30)
COLOR_GRID = (40, 40, 50)
COLOR_OBSTACLE = (100, 100, 110)
COLOR_GNOME = (220, 50, 50) # Rojo
COLOR_HOME = (200, 200, 200, 100) # Blanco semitransparente
COLOR_ROBOT = (50, 150, 250) # Azul
COLOR_TEXT = (240, 240, 240)
COLOR_BATTERY_GREEN = (50, 200, 50)
COLOR_BATTERY_ORANGE = (250, 160, 50)
COLOR_BATTERY_RED = (220, 50, 50)
COLOR_VISION = (50, 150, 250, 50) # Azul del robot, con transparencia

# --- Configuración de la Simulación ---
CONFIG = {
    "num_gnomes": 8, "num_obstacles": 15, "animation_speed": 3.0,
    "battery_capacity": 100.0, "battery_low_threshold": 25.0,
    "drain_base": 0.05, "drain_move": 2.0, "drain_turn": 1.0, "charge_rate": 10.0,
}


# --- Algoritmo de Búsqueda de Caminos A* (sin cambios) ---
class Node:
    def __init__(self, position, parent=None):
        self.position, self.parent = position, parent
        self.g, self.h, self.f = 0, 0, 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f
    def __repr__(self): return f"Node({self.position})"

def find_path_astar(grid, start, end):
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
            if not (0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0])) or \
               grid[neighbor_pos[0]][neighbor_pos[1]] == 1 or neighbor_pos in closed_list:
                continue
            
            neighbor_node = Node(neighbor_pos, current_node)
            neighbor_node.g = current_node.g + 1
            neighbor_node.h = abs(neighbor_pos[0] - end_node.position[0]) + abs(neighbor_pos[1] - end_node.position[1])
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            if any(n for n in open_list if neighbor_node == n and neighbor_node.g >= n.g):
                continue
            heapq.heappush(open_list, neighbor_node)
    return None

class World:
    # (Sin cambios en la lógica interna)
    def __init__(self):
        self.width, self.height = GRID_WIDTH, GRID_HEIGHT
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.gnomes = []
        self.home_base = (0, 0)
        self.grid[self.home_base[0]][self.home_base[1]] = 3
        self.generate_obstacles_and_gnomes()

    def generate_obstacles_and_gnomes(self):
        self.grid = [[0 if self.grid[r][c] != 3 else 3 for c in range(self.width)] for r in range(self.height)]
        self.gnomes = []
        
        for _ in range(CONFIG["num_obstacles"]):
            while True:
                r, c = random.randint(0, self.height - 1), random.randint(0, self.width - 1)
                if self.grid[r][c] == 0 and math.dist((r, c), self.home_base) > 2:
                    self.grid[r][c] = 1
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
        # ESTADOS Y ACCIONES EN ESPAÑOL
        self.action, self.path = 'ESPERANDO', []
        self.state = 'BUSCANDO'
        
        self.animation_progress = 0.0
        self.battery, self.inventory = CONFIG["battery_capacity"], []
        self.discovered_gnomes = set()
        self.patrol_points = self.generate_patrol_points()
        self.patrol_index, self.current_target_pos = 0, None
        self.saved_state, self.saved_target_pos, self.saved_path = None, None, None
        self.target_theta = 0.0

        self.view_pattern = {
            0: [(0, 1), (0, 2), (1, 1), (-1, 1)],         # Este
            math.pi/2: [(-1, 0), (-2, 0), (-1, 1), (-1, -1)], # Norte
            math.pi: [(0, -1), (0, -2), (1, -1), (-1, -1)], # Oeste
            -math.pi/2: [(1, 0), (2, 0), (1, 1), (1, -1)], # Sur
        }

    def generate_patrol_points(self):
        points = []
        for r in range(GRID_HEIGHT):
            row_points = range(GRID_WIDTH) if r % 2 == 0 else range(GRID_WIDTH - 1, -1, -1)
            for c in row_points: points.append((r, c))
        return points

    def update_vision(self, world):
        closest_theta_key = min(self.view_pattern.keys(), key=lambda k: abs(k - self.theta))
        for dr, dc in self.view_pattern.get(closest_theta_key, []):
            check_r, check_c = self.row + dr, self.col + dc
            if 0 <= check_r < world.height and 0 <= check_c < world.width:
                if world.grid[check_r][check_c] == 2 and (check_r, check_c) not in self.discovered_gnomes:
                    self.discovered_gnomes.add((check_r, check_c))
                    return (check_r, check_c)
        return None

    def update(self, world, dt):
        self.update_logic(world, dt)
        self.execute_movement(dt)

    def execute_movement(self, dt):
        if self.action == 'ESPERANDO': return

        drain = CONFIG["drain_base"] * dt
        if self.action == 'GIRANDO': drain += CONFIG["drain_turn"] * dt
        elif self.action == 'MOVIENDOSE': drain += CONFIG["drain_move"] * dt
        self.battery = max(0, self.battery - drain)

        self.animation_progress += CONFIG["animation_speed"] * dt
        
        if self.animation_progress >= 1.0:
            self.animation_progress %= 1.0 # Usamos módulo para evitar saltos si dt es grande
            
            if self.action == 'GIRANDO':
                self.theta = self.target_theta
            elif self.action == 'MOVIENDOSE' and self.path:
                self.row, self.col = self.path.pop(0)

            if not self.path:
                self.action = 'ESPERANDO'
            else:
                self.start_next_path_step()

    def start_next_path_step(self):
        if not self.path:
            self.action = 'ESPERANDO'
            return
            
        next_pos = self.path[0]
        dr, dc = next_pos[0] - self.row, next_pos[1] - self.col
        
        self.target_theta = math.atan2(-dr, dc)
        angle_diff = (self.target_theta - self.theta + math.pi) % (2 * math.pi) - math.pi
        
        if abs(angle_diff) > 0.1:
            self.action = 'GIRANDO'
        else:
            self.action = 'MOVIENDOSE'

    def update_logic(self, world, dt):
        if self.battery <= CONFIG["battery_low_threshold"] and self.state not in ['VOLVIENDO_A_CASA', 'CARGANDO']:
            self.saved_state, self.saved_target_pos, self.saved_path = self.state, self.current_target_pos, self.path
            self.current_target_pos = world.home_base
            self.path = find_path_astar(world.grid, (self.row, self.col), self.current_target_pos)
            self.state = 'VOLVIENDO_A_CASA'
            if self.path: self.start_next_path_step()

        if self.state == 'BUSCANDO':
            # MEJORA: El robot ahora "mira" en cada fotograma, incluso mientras se mueve.
            found_gnome = self.update_vision(world)
            if found_gnome:
                # Si encuentra un gnomo, interrumpe el patrullaje y va por él.
                self.state = 'YENDO_AL_GNOMO'
                self.current_target_pos = found_gnome
                self.path = find_path_astar(world.grid, (self.row, self.col), self.current_target_pos)
                if self.path: self.start_next_path_step()
                return # Salimos para procesar el nuevo estado en el siguiente ciclo

            # Si está esperando (sin ruta), busca una nueva ruta de patrullaje.
            if self.action == 'ESPERANDO':
                if self.patrol_index < len(self.patrol_points):
                    next_patrol = self.patrol_points[self.patrol_index]
                    if (self.row, self.col) != next_patrol:
                        self.path = find_path_astar(world.grid, (self.row, self.col), next_patrol)
                    self.patrol_index += 1
                else: # Terminó de patrullar
                    self.state = 'TERMINADO'
        
        # El resto de la lógica solo se ejecuta si el robot está en espera.
        if self.action != 'ESPERANDO': return

        if self.state == 'YENDO_AL_GNOMO':
            gnome_pos = self.current_target_pos
            if gnome_pos in world.gnomes:
                world.gnomes.remove(gnome_pos)
                world.grid[gnome_pos[0]][gnome_pos[1]] = 0
                self.inventory.append(gnome_pos)
            self.state = 'YENDO_A_BASE'
            self.current_target_pos = world.home_base
            self.path = find_path_astar(world.grid, (self.row, self.col), self.current_target_pos)

        elif self.state == 'YENDO_A_BASE':
            if self.inventory: self.inventory.pop(0)
            self.state = 'BUSCANDO'

        elif self.state == 'VOLVIENDO_A_CASA':
            self.state = 'CARGANDO'
            
        elif self.state == 'CARGANDO':
            self.battery += CONFIG["charge_rate"] * dt
            if self.battery >= CONFIG["battery_capacity"]:
                self.battery = CONFIG["battery_capacity"]
                self.state, self.current_target_pos, self.path = self.saved_state, self.saved_target_pos, self.saved_path
                self.saved_state, self.saved_target_pos, self.saved_path = None, None, None

        if self.action == 'ESPERANDO' and self.path: self.start_next_path_step()

# --- VISTA DE PYGAME ---
def draw_world(screen, world):
    for r in range(world.height):
        for c in range(world.width):
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            cell_type = world.grid[r][c]
            if cell_type == 1: pygame.draw.rect(screen, COLOR_OBSTACLE, rect)
            elif cell_type == 2: pygame.draw.circle(screen, COLOR_GNOME, rect.center, int(CELL_SIZE * 0.3))
            elif cell_type == 3: pygame.draw.rect(screen, COLOR_HOME, rect, border_radius=5)

def draw_grid(screen):
    for x in range(0, SCREEN_WIDTH, CELL_SIZE):
        pygame.draw.line(screen, COLOR_GRID, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, CELL_SIZE):
        pygame.draw.line(screen, COLOR_GRID, (0, y), (SCREEN_WIDTH, y))

def draw_vision_cone(screen, robot):
    vision_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
    closest_theta_key = min(robot.view_pattern.keys(), key=lambda k: abs(k - robot.theta))
    
    for dr, dc in robot.view_pattern.get(closest_theta_key, []):
        r, c = robot.row + dr, robot.col + dc
        if 0 <= r < GRID_HEIGHT and 0 <= c < GRID_WIDTH:
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(vision_surface, COLOR_VISION, rect)
            
    screen.blit(vision_surface, (0, 0))

def draw_robot(screen, robot):
    start_px = (robot.col * CELL_SIZE + CELL_SIZE / 2, robot.row * CELL_SIZE + CELL_SIZE / 2)
    end_px = start_px
    if robot.action == 'MOVIENDOSE' and robot.path:
        next_r, next_c = robot.path[0]
        end_px = (next_c * CELL_SIZE + CELL_SIZE / 2, next_r * CELL_SIZE + CELL_SIZE / 2)
    
    t = robot.animation_progress
    pos_x = start_px[0] * (1 - t) + end_px[0] * t
    pos_y = start_px[1] * (1 - t) + end_px[1] * t

    target_theta_interp = robot.theta
    if robot.action == 'GIRANDO':
        target_theta_interp = robot.target_theta
    
    angle_diff = (target_theta_interp - robot.theta + math.pi) % (2 * math.pi) - math.pi
    interp_angle = robot.theta + angle_diff * t
    
    pygame.draw.circle(screen, COLOR_ROBOT, (pos_x, pos_y), int(CELL_SIZE * 0.4))
    
    line_end_x = pos_x + math.cos(interp_angle) * (CELL_SIZE * 0.35)
    line_end_y = pos_y - math.sin(interp_angle) * (CELL_SIZE * 0.35)
    pygame.draw.line(screen, COLOR_TEXT, (pos_x, pos_y), (line_end_x, line_end_y), 3)

def draw_ui(screen, robot, font, small_font):
    ui_rect = pygame.Rect(SCREEN_WIDTH, 0, UI_WIDTH, SCREEN_HEIGHT)
    pygame.draw.rect(screen, (50, 50, 60), ui_rect)
    
    y = 20
    def draw_text(text, font_obj, pos, color=COLOR_TEXT):
        text_surf = font_obj.render(text, True, color)
        screen.blit(text_surf, pos)
        return text_surf.get_height()

    # --- TEXTOS EN ESPAÑOL ---
    y += draw_text("ESTADO DEL ROBOT", font, (SCREEN_WIDTH + 20, y)) + 10
    y += draw_text(f"Estado: {robot.state}", small_font, (SCREEN_WIDTH + 25, y))
    y += draw_text(f"Acción: {robot.action}", small_font, (SCREEN_WIDTH + 25, y)) + 20
    
    y += draw_text("Batería", small_font, (SCREEN_WIDTH + 25, y)) + 5
    pct = robot.battery / CONFIG["battery_capacity"] if CONFIG["battery_capacity"] > 0 else 0
    bar_w, bar_h = UI_WIDTH - 50, 20
    bar_color = COLOR_BATTERY_RED if pct < 0.25 else COLOR_BATTERY_ORANGE if pct < 0.5 else COLOR_BATTERY_GREEN
    pygame.draw.rect(screen, (20,20,30), (SCREEN_WIDTH + 25, y, bar_w, bar_h))
    pygame.draw.rect(screen, bar_color, (SCREEN_WIDTH + 25, y, bar_w * pct, bar_h))
    y += bar_h + 5
    y += draw_text(f"{robot.battery:.1f} / {CONFIG['battery_capacity']:.1f}", small_font, (SCREEN_WIDTH + 25, y)) + 30

    y += draw_text("Gnomos Descubiertos", font, (SCREEN_WIDTH + 20, y)) + 10
    for i, pos in enumerate(list(robot.discovered_gnomes)):
        y += draw_text(f"- Gnomo en ({pos[0]}, {pos[1]})", small_font, (SCREEN_WIDTH + 25, y))
        if y > SCREEN_HEIGHT - 40:
            draw_text("...", small_font, (SCREEN_WIDTH + 25, y))
            break

# --- Bucle Principal del Juego ---
if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode((TOTAL_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Simulación de Robot 2D en Rejilla") # Título en español
    clock = pygame.time.Clock()
    
    font = pygame.font.Font(None, 36)
    small_font = pygame.font.Font(None, 28)

    world = World()
    robot = SimulatedRobot()
    simulation_running = True
    
    running = True
    while running:
        dt_sec = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    simulation_running = not simulation_running
                if event.key == pygame.K_r:
                    world = World()
                    robot = SimulatedRobot()
                    simulation_running = True

        if simulation_running:
            robot.update(world, dt_sec)
            if robot.state == 'TERMINADO' or robot.battery <= 0:
                simulation_running = False

        screen.fill(COLOR_BG)
        draw_world(screen, world)
        draw_grid(screen)
        
        draw_vision_cone(screen, robot)
        draw_robot(screen, robot)
        
        draw_ui(screen, robot, font, small_font)
        
        pygame.display.flip()

    pygame.quit()