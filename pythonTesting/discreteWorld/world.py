# world.py
import math
import random
from constants import CONFIG, MEM_UNKNOWN, MEM_OBSTACLE, data_lock, shared_data
from utils import find_path_astar, bresenham_line

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
