# SimulatedRobot.py

import math

class SimulatedRobot:
    """Encapsula toda la lógica, física y estado del robot."""
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """Resetea el estado del robot a su condición inicial."""
        self.x, self.y, self.theta = 0.5, 0.5, 0.0
        self.linear_velocity, self.angular_velocity = 0.0, 0.0
        self.camera_fov_rad = math.radians(self.config["camera_fov_degrees"])
        self.camera_range = self.config["camera_range"]
        self.battery = self.config["battery_capacity"]
        self.state = 'SEARCHING'
        self.targeted_gnomes = []
        self.newly_discovered_gnomes = []
        self.inventory = []
        self.patrol_points = self._generate_patrol_points()
        self.target_index, self.current_target = 0, None
        self.saved_state, self.saved_target = None, None
        self.saved_patrol_index, self.saved_patrol_target = None, 0

    def _generate_patrol_points(self, step=1.5):
        points, margin, x, direction = [], step/2.0, step/2.0, 1
        while x < 10.0:
            y1, y2 = (margin, 10.0 - margin) if direction == 1 else (10.0 - margin, margin)
            points.extend([(x, y1), (x, y2)])
            direction *= -1
            x += step
        return points

    def update(self, world, dt):
        """Método principal que se llama en cada paso de la simulación."""
        self._update_logic(world)
        self._update_physics_and_battery(dt)

    def _update_physics_and_battery(self, dt):
        drain = self.config["drain_base"] * dt
        if self.linear_velocity > 0.01: drain += self.config["drain_move"] * dt
        if abs(self.angular_velocity) > 0.01: drain += self.config["drain_turn"] * dt
        self.battery = max(0, self.battery - drain)
        
        self.theta += self.angular_velocity * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # --- CAMBIO CLAVE: Ajusta el ángulo a 90° exactos cuando casi no hay giro ---
        if abs(self.angular_velocity) < 0.1:
            self.theta = round(self.theta / (math.pi / 2)) * (math.pi / 2)
        # --------------------------------------------------------------------------
        
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
    
    def _update_vision(self, world):
        for gx, gy in world.gnomes:
            is_already_targeted = any(math.dist((gx, gy), f) < 0.5 for f in self.targeted_gnomes)
            if not is_already_targeted and math.dist((self.x, self.y), (gx, gy)) <= self.camera_range:
                angle_to_gnome = math.atan2(gy - self.y, gx - self.x)
                angle_diff = (angle_to_gnome - self.theta + math.pi) % (2 * math.pi) - math.pi
                if abs(angle_diff) <= self.camera_fov_rad / 2.0:
                    self.targeted_gnomes.append((gx, gy))
                    self.newly_discovered_gnomes.append((gx, gy))
                    return (gx, gy)
        return None
    
    # --- CAMBIO CLAVE: Nuevo método de navegación en 90 grados ---
    def _navigate_grid_style(self):
        if self.current_target is None:
            self.linear_velocity, self.angular_velocity = 0.0, 0.0
            return
        
        dx = self.current_target[0] - self.x
        dy = self.current_target[1] - self.y
        dist = math.sqrt(dx**2 + dy**2)
        
        target_angle = self.theta
        align_tol = 0.1

        # 1. Prioriza el alineamiento y movimiento en el eje X
        if abs(dx) > align_tol:
            target_angle = 0 if dx > 0 else math.pi
        # 2. Solo si ya está alineado en X, se enfoca en el eje Y
        elif abs(dy) > align_tol:
            target_angle = math.pi/2 if dy > 0 else -math.pi/2

        angle_diff = (target_angle - self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Si no está alineado, GIRA. Si está alineado, AVANZA.
        if abs(angle_diff) > 0.05:
            self.linear_velocity = 0.0
            self.angular_velocity = self.config["Kp_turn"] * angle_diff
        else:
            self.angular_velocity = 0.0
            self.linear_velocity = self.config["robot_speed"] * min(1.0, dist / 0.5)
    # ----------------------------------------------------------------

    def _update_logic(self, world):
        if self.battery <= self.config["battery_low_threshold"] and self.state not in ['RETURNING_HOME', 'CHARGING']:
            self.saved_state, self.saved_target = self.state, self.current_target
            self.current_target, self.state = (0.5, 0.5), 'RETURNING_HOME'
        
        if self.state in ['SEARCHING','NAVIGATING_TO_GNOME','NAVIGATING_TO_DROPOFF','RETURNING_HOME']:
            # --- CAMBIO CLAVE: Se llama al nuevo método de navegación ---
            self._navigate_grid_style()
            # -----------------------------------------------------------

        if self.state == 'SEARCHING':
            if (found_gnome := self._update_vision(world)):
                self.saved_patrol_index, self.saved_patrol_target = self.target_index, self.current_target
                self.current_target, self.state = found_gnome, 'NAVIGATING_TO_GNOME'
                return

            if self.current_target is None and self.target_index < len(self.patrol_points):
                self.current_target = self.patrol_points[self.target_index]
            elif self.current_target and math.dist((self.x, self.y), self.current_target) < 0.3:
                self.target_index += 1
                self.current_target = None
                if self.target_index >= len(self.patrol_points):
                    self.state = 'FINISHED'
        
        elif self.state == 'NAVIGATING_TO_GNOME' and self.current_target and math.dist((self.x, self.y), self.current_target) < 0.3:
            if self.current_target in world.gnomes:
                world.gnomes.remove(self.current_target)
            self.targeted_gnomes = [g for g in self.targeted_gnomes if math.dist(g, self.current_target) > 0.1]
            self.inventory.append(self.current_target)
            self.current_target, self.state = (0.5, 0.5), 'NAVIGATING_TO_DROPOFF'
        
        elif self.state == 'NAVIGATING_TO_DROPOFF' and self.current_target and math.dist((self.x, self.y), self.current_target) < 0.3:
            self.inventory.pop(0)
            self.target_index, self.current_target = self.saved_patrol_index, self.saved_patrol_target
            self.state = 'SEARCHING'
        
        elif self.state == 'RETURNING_HOME' and math.dist((self.x, self.y), (0.5, 0.5)) < 0.2:
            self.linear_velocity, self.angular_velocity, self.state = 0.0, 0.0, 'CHARGING'
        
        elif self.state == 'CHARGING':
            self.battery += self.config["charge_rate"] * self.config["time_step"]
            if self.battery >= self.config["battery_capacity"]:
                self.battery = self.config["battery_capacity"]
                self.state, self.current_target = self.saved_state, self.saved_target
        
        elif self.state == 'FINISHED':
            self.linear_velocity, self.angular_velocity = 0.0, 0.0






