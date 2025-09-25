# pygame_sim_2d.py

import pygame
import math
import random
import heapq

# --- Pygame & Display Configuration ---
CELL_SIZE = 60
GRID_WIDTH, GRID_HEIGHT = 10, 10
UI_WIDTH = 300
SCREEN_WIDTH = GRID_WIDTH * CELL_SIZE
SCREEN_HEIGHT = GRID_HEIGHT * CELL_SIZE
TOTAL_WIDTH = SCREEN_WIDTH + UI_WIDTH

# Colors
COLOR_BG = (20, 20, 30)
COLOR_GRID = (40, 40, 50)
COLOR_OBSTACLE = (100, 100, 110)
COLOR_GNOME = (220, 50, 50) # Red
COLOR_HOME = (200, 200, 200, 100) # Semi-transparent white
COLOR_ROBOT = (50, 150, 250) # Blue
COLOR_TEXT = (240, 240, 240)
COLOR_BATTERY_GREEN = (50, 200, 50)
COLOR_BATTERY_ORANGE = (250, 160, 50)
COLOR_BATTERY_RED = (220, 50, 50)

# --- Simulation Configuration (Same as before) ---
CONFIG = {
    "num_gnomes": 8, "num_obstacles": 15, "animation_speed": 2.5,
    "battery_capacity": 100.0, "battery_low_threshold": 25.0,
    "drain_base": 0.05, "drain_move": 2.0, "drain_turn": 1.0, "charge_rate": 10.0,
}

# -------------------------------------------------------------------------- #
# --- LOGIC & MODEL (These sections are UNCHANGED from the discrete sim) --- #
# -------------------------------------------------------------------------- #

# --- A* Pathfinding Algorithm ---
class Node:
    def __init__(self, position, parent=None):
        self.position, self.parent = position, parent
        self.g, self.h, self.f = 0, 0, 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

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

            if any(n for n in open_list if neighbor_node == n and neighbor_node.g > n.g):
                continue
            heapq.heappush(open_list, neighbor_node)
    return None

# --- World and Robot Classes ---
class World:
    def __init__(self):
        self.width, self.height = GRID_WIDTH, GRID_HEIGHT
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.gnomes = []
        self.home_base = (0, 0)
        self.grid[self.home_base[0]][self.home_base[1]] = 3
        self.generate_obstacles_and_gnomes()

    def generate_obstacles_and_gnomes(self):
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
        self.action, self.animation_progress, self.path = 'IDLE', 0.0, []
        self.battery, self.inventory, self.state = CONFIG["battery_capacity"], [], 'SEARCHING'
        self.discovered_gnomes, self.newly_discovered_gnomes = set(), []
        self.patrol_points = self.generate_patrol_points()
        self.patrol_index, self.current_target_pos = 0, None
        self.saved_state, self.saved_target_pos, self.saved_path = None, None, None

    def generate_patrol_points(self):
        points = []
        for r in range(GRID_HEIGHT):
            if r % 2 == 0:
                for c in range(GRID_WIDTH): points.append((r, c))
            else:
                for c in range(GRID_WIDTH - 1, -1, -1): points.append((r, c))
        return points

    def update_vision(self, world):
        view_pattern = {
            0: [(0, 1), (0, 2), (1, 1), (-1, 1)],  # East
            math.pi/2: [(-1, 0), (-2, 0), (-1, 1), (-1, -1)],  # North
            math.pi: [(0, -1), (0, -2), (1, -1), (-1, -1)],  # West
            -math.pi/2: [(1, 0), (2, 0), (1, 1), (1, -1)],  # South
        }
        closest_theta = min(view_pattern.keys(), key=lambda k: abs(k - self.theta))
        for dr, dc in view_pattern.get(closest_theta, []):
            check_r, check_c = self.row + dr, self.col + dc
            if 0 <= check_r < world.height and 0 <= check_c < world.width:
                if world.grid[check_r][check_c] == 2 and (check_r, check_c) not in self.discovered_gnomes:
                    self.discovered_gnomes.add((check_r, check_c))
                    self.newly_discovered_gnomes.append((check_r, check_c))
                    return (check_r, check_c)
        return None

    def update(self, world, dt):
        self.update_logic(world)
        self.execute_movement(dt)

    def execute_movement(self, dt):
        drain = CONFIG["drain_base"] * dt
        if self.action == 'TURNING': drain += CONFIG["drain_turn"] * dt
        elif self.action == 'MOVING': drain += CONFIG["drain_move"] * dt
        self.battery = max(0, self.battery - drain)

        if self.action != 'IDLE':
            self.animation_progress += CONFIG["animation_speed"] * dt
            if self.animation_progress >= 1.0:
                self.animation_progress = 0.0
                if self.action == 'MOVING' and self.path:
                    self.row, self.col = self.path.pop(0)
                if not self.path: self.action = 'IDLE'
                else: self.start_next_path_step()

    def start_next_path_step(self):
        if not self.path:
            self.action = 'IDLE'
            return
        next_pos = self.path[0]
        dr, dc = next_pos[0] - self.row, next_pos[1] - self.col
        target_theta = math.atan2(-dr, dc)
        angle_diff = (target_theta - self.theta + math.pi) % (2*math.pi) - math.pi
        self.action = 'TURNING' if abs(angle_diff) > 0.1 else 'MOVING'

    def update_logic(self, world):
        if self.battery <= CONFIG["battery_low_threshold"] and self.state not in ['RETURNING_HOME', 'CHARGING']:
            self.saved_state, self.saved_target_pos, self.saved_path = self.state, self.current_target_pos, self.path
            self.current_target_pos = world.home_base
            self.path = find_path_astar(world.grid, (self.row, self.col), self.current_target_pos)
            self.state = 'RETURNING_HOME'
            if self.path: self.start_next_path_step()

        if self.action != 'IDLE': return

        if self.state == 'SEARCHING':
            if (found := self.update_vision(world)):
                self.state, self.current_target_pos = 'NAVIGATING_TO_GNOME', found
                self.path = find_path_astar(world.grid, (self.row, self.col), self.current_target_pos)
            elif not self.path and self.patrol_index < len(self.patrol_points):
                next_patrol = self.patrol_points[self.patrol_index]
                if (self.row, self.col) != next_patrol:
                    self.path = find_path_astar(world.grid, (self.row, self.col), next_patrol)
                self.patrol_index += 1
            elif not self.path: self.state = 'FINISHED'

        elif self.state == 'NAVIGATING_TO_GNOME' and not self.path:
            gnome_pos = self.current_target_pos
            if gnome_pos in world.gnomes:
                world.gnomes.remove(gnome_pos)
                world.grid[gnome_pos[0]][gnome_pos[1]] = 0
                self.inventory.append(gnome_pos)
            self.state, self.current_target_pos = 'NAVIGATING_TO_DROPOFF', world.home_base
            self.path = find_path_astar(world.grid, (self.row, self.col), self.current_target_pos)

        elif self.state == 'NAVIGATING_TO_DROPOFF' and not self.path:
            if self.inventory: self.inventory.pop(0)
            self.state = 'SEARCHING'

        elif self.state == 'RETURNING_HOME' and not self.path: self.state = 'CHARGING'
        elif self.state == 'CHARGING':
            self.battery += CONFIG["charge_rate"] * dt_sec
            if self.battery >= CONFIG["battery_capacity"]:
                self.battery = CONFIG["battery_capacity"]
                self.state, self.current_target_pos, self.path = self.saved_state, self.saved_target_pos, self.saved_path

        if self.action == 'IDLE' and self.path: self.start_next_path_step()

# ------------------------------------------------------------- #
# --- PYGAME VIEW (This section is the new rendering engine) --- #
# ------------------------------------------------------------- #

def draw_world(screen, world):
    for r in range(world.height):
        for c in range(world.width):
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            cell_type = world.grid[r][c]
            if cell_type == 1: # Obstacle
                pygame.draw.rect(screen, COLOR_OBSTACLE, rect)
            elif cell_type == 2: # Gnome
                pygame.draw.circle(screen, COLOR_GNOME, rect.center, int(CELL_SIZE * 0.3))
            elif cell_type == 3: # Home
                pygame.draw.rect(screen, COLOR_HOME, rect, border_radius=5)

def draw_grid(screen):
    for x in range(0, SCREEN_WIDTH, CELL_SIZE):
        pygame.draw.line(screen, COLOR_GRID, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, CELL_SIZE):
        pygame.draw.line(screen, COLOR_GRID, (0, y), (SCREEN_WIDTH, y))

def draw_robot(screen, robot):
    # Interpolate position for smooth animation
    start_px = (robot.col * CELL_SIZE + CELL_SIZE / 2, robot.row * CELL_SIZE + CELL_SIZE / 2)
    end_px = start_px
    if robot.action == 'MOVING' and robot.path:
        next_r, next_c = robot.path[0]
        end_px = (next_c * CELL_SIZE + CELL_SIZE / 2, next_r * CELL_SIZE + CELL_SIZE / 2)
    
    t = robot.animation_progress
    pos_x = start_px[0] * (1 - t) + end_px[0] * t
    pos_y = start_px[1] * (1 - t) + end_px[1] * t

    # Interpolate angle for smooth turning
    target_theta = robot.theta
    if robot.action == 'TURNING' and robot.path:
        next_r, next_c = robot.path[0]
        target_theta = math.atan2(-(next_r - robot.row), next_c - robot.col)
    
    # Smallest angle difference for smooth rotation
    angle_diff = (target_theta - robot.theta + math.pi) % (2 * math.pi) - math.pi
    interp_angle = robot.theta + angle_diff * min(1.0, t * 2) # Turn faster than move
    if robot.action == 'TURNING' and t >= 1.0: robot.theta = target_theta # Snap to final angle

    # Draw robot body
    pygame.draw.circle(screen, COLOR_ROBOT, (pos_x, pos_y), int(CELL_SIZE * 0.4))
    
    # Draw orientation line
    line_end_x = pos_x + math.cos(interp_angle) * (CELL_SIZE * 0.35)
    line_end_y = pos_y - math.sin(interp_angle) * (CELL_SIZE * 0.35) # Pygame Y is inverted
    pygame.draw.line(screen, COLOR_TEXT, (pos_x, pos_y), (line_end_x, line_end_y), 3)

def draw_ui(screen, robot, font, small_font):
    ui_rect = pygame.Rect(SCREEN_WIDTH, 0, UI_WIDTH, SCREEN_HEIGHT)
    pygame.draw.rect(screen, (50, 50, 60), ui_rect)
    
    y = 20
    # Helper to draw text
    def draw_text(text, font_obj, pos, color=COLOR_TEXT):
        text_surf = font_obj.render(text, True, color)
        screen.blit(text_surf, pos)
        return text_surf.get_height()

    y += draw_text("ROBOT STATUS", font, (SCREEN_WIDTH + 20, y)) + 10
    y += draw_text(f"State: {robot.state}", small_font, (SCREEN_WIDTH + 25, y))
    y += draw_text(f"Action: {robot.action}", small_font, (SCREEN_WIDTH + 25, y)) + 20
    
    # Battery Bar
    y += draw_text("Battery", small_font, (SCREEN_WIDTH + 25, y)) + 5
    pct = robot.battery / CONFIG["battery_capacity"]
    bar_w, bar_h = UI_WIDTH - 50, 20
    bar_color = COLOR_BATTERY_RED if pct < 0.25 else COLOR_BATTERY_ORANGE if pct < 0.5 else COLOR_BATTERY_GREEN
    pygame.draw.rect(screen, (20,20,30), (SCREEN_WIDTH + 25, y, bar_w, bar_h))
    pygame.draw.rect(screen, bar_color, (SCREEN_WIDTH + 25, y, bar_w * pct, bar_h))
    y += bar_h + 5
    y += draw_text(f"{robot.battery:.1f} / {CONFIG['battery_capacity']:.1f}", small_font, (SCREEN_WIDTH + 25, y)) + 30

    # Discovered Gnomes
    y += draw_text("Discovered Gnomes", font, (SCREEN_WIDTH + 20, y)) + 10
    for i, pos in enumerate(robot.discovered_gnomes):
        y += draw_text(f"- Gnome at ({pos[0]}, {pos[1]})", small_font, (SCREEN_WIDTH + 25, y))
        if y > SCREEN_HEIGHT - 40: break # Don't draw off-screen

# --- Main Game Loop ---
if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode((TOTAL_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("2D Grid Robot Simulation")
    clock = pygame.time.Clock()
    
    font = pygame.font.Font(None, 36)
    small_font = pygame.font.Font(None, 28)

    world = World()
    robot = SimulatedRobot()
    simulation_running = False
    
    running = True
    while running:
        # Get delta time in seconds for frame-rate independent logic
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
                    simulation_running = False

        # --- UPDATE ---
        if simulation_running:
            robot.update(world, dt_sec)
            if robot.state == 'FINISHED' or robot.battery <= 0:
                simulation_running = False

        # --- DRAW ---
        screen.fill(COLOR_BG)
        draw_world(screen, world)
        draw_grid(screen)
        draw_robot(screen, robot)
        draw_ui(screen, robot, font, small_font)
        
        pygame.display.flip()

    pygame.quit()