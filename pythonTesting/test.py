# vpython_sim_interactive.py

from vpython import *
import math
import random

# --- Default Configuration ---
CONFIG = {
    "num_gnomes": 8, "robot_speed": 1.5, "Kp_turn": 4.0, "camera_fov_degrees": 70,
    "battery_capacity": 100.0, "battery_low_threshold": 25.0, "time_step": 0.01,
    "camera_range": 3.0, "drain_base": 0.1, "drain_move": 1.0, "drain_turn": 0.5, "charge_rate": 10.0,
}

# --- Global Simulation State ---
simulation_running = False
world, robot = None, None
vpython_objects, run_button = {}, None
discovered_gnome_count = 0 # Counter for labeling discovered gnomes

class World:
    def __init__(self):
        self.width, self.height = 10.0, 10.0
        self.gnomes = [(random.uniform(1.0, self.width - 1.0), random.uniform(1.0, self.height - 1.0)) for _ in range(CONFIG["num_gnomes"])]

class SimulatedRobot:
    def __init__(self):
        self.x, self.y, self.theta = 0.5, 0.5, 0.0
        self.linear_velocity, self.angular_velocity = 0.0, 0.0
        self.camera_fov_rad = math.radians(CONFIG["camera_fov_degrees"])
        self.camera_range = CONFIG["camera_range"]
        self.battery = CONFIG["battery_capacity"]
        self.saved_state, self.saved_target = None, None
        self.saved_patrol_target, self.saved_patrol_index = None, 0
        self.state, self.targeted_gnomes = 'SEARCHING', []
        self.newly_discovered_gnomes = [] # <<< NEW: Mailbox for newly found gnomes
        self.patrol_points = self.generate_patrol_points()
        self.target_index, self.current_target, self.inventory = 0, None, []

    def generate_patrol_points(self, step=1.5):
        points, margin, x, direction = [], step/2.0, step/2.0, 1
        while x < 10.0:
            y1, y2 = (margin, 10.0 - margin) if direction == 1 else (10.0 - margin, margin)
            points.extend([(x, y1), (x, y2)]); direction *= -1; x += step
        return points

    def update_physics_and_battery(self, dt):
        drain = CONFIG["drain_base"] * dt
        if self.linear_velocity > 0.01: drain += CONFIG["drain_move"] * dt
        if abs(self.angular_velocity) > 0.01: drain += CONFIG["drain_turn"] * dt
        self.battery = max(0, self.battery - drain)
        self.theta = math.atan2(math.sin(self.theta + self.angular_velocity*dt), math.cos(self.theta + self.angular_velocity*dt))
        if abs(self.angular_velocity) < 0.1: self.theta = round(self.theta / (math.pi/2)) * (math.pi/2)
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
    
    def update_vision(self, world):
        for gx, gy in world.gnomes:
            if not any(math.dist((gx, gy), f) < 0.5 for f in self.targeted_gnomes) and math.dist((self.x,self.y), (gx,gy)) <= self.camera_range:
                angle_to_gnome = math.atan2(gy - self.y, gx - self.x)
                angle_diff = (angle_to_gnome - self.theta + math.pi) % (2*math.pi) - math.pi
                if abs(angle_diff) <= self.camera_fov_rad / 2.0:
                    self.targeted_gnomes.append((gx, gy))
                    self.newly_discovered_gnomes.append((gx, gy)) # <<< MODIFIED: Add to mailbox
                    return (gx, gy)
        return None
    
    def navigate_grid_style(self):
        if self.current_target is None: self.linear_velocity, self.angular_velocity = 0.0, 0.0; return
        dx, dy = self.current_target[0] - self.x, self.current_target[1] - self.y
        dist = math.sqrt(dx**2 + dy**2); target_angle, align_tol = self.theta, 0.1
        if abs(dx) > align_tol: target_angle = 0 if dx > 0 else math.pi
        elif abs(dy) > align_tol: target_angle = math.pi/2 if dy > 0 else -math.pi/2
        angle_diff = (target_angle - self.theta + math.pi) % (2*math.pi) - math.pi
        if abs(angle_diff) > 0.05: self.linear_velocity, self.angular_velocity = 0.0, CONFIG["Kp_turn"]*angle_diff
        else: self.angular_velocity, self.linear_velocity = 0.0, CONFIG["robot_speed"]*min(1.0, dist/0.5)

    def update_logic(self, world):
        if self.battery <= CONFIG["battery_low_threshold"] and self.state not in ['RETURNING_HOME', 'CHARGING']:
            self.saved_state, self.saved_target = self.state, self.current_target
            self.current_target, self.state = (0.5, 0.5), 'RETURNING_HOME'
        if self.state in ['SEARCHING','NAVIGATING_TO_GNOME','NAVIGATING_TO_DROPOFF','RETURNING_HOME']: self.navigate_grid_style()
        if self.state == 'SEARCHING':
            if (found_gnome := self.update_vision(world)):
                self.saved_patrol_index, self.saved_patrol_target = self.target_index, self.current_target
                self.current_target, self.state = found_gnome, 'NAVIGATING_TO_GNOME'; return
            if self.current_target is None and self.target_index < len(self.patrol_points): self.current_target = self.patrol_points[self.target_index]
            elif self.current_target and math.dist((self.x, self.y), self.current_target) < 0.3:
                self.target_index += 1; self.current_target = None
                if self.target_index >= len(self.patrol_points): self.state = 'FINISHED'
        elif self.state == 'NAVIGATING_TO_GNOME' and self.current_target and math.dist((self.x, self.y), self.current_target) < 0.3:
            if self.current_target in world.gnomes: world.gnomes.remove(self.current_target)
            self.targeted_gnomes = [g for g in self.targeted_gnomes if math.dist(g, self.current_target) > 0.1]
            self.inventory.append(self.current_target); self.current_target, self.state = (0.5, 0.5), 'NAVIGATING_TO_DROPOFF'
        elif self.state == 'NAVIGATING_TO_DROPOFF' and self.current_target and math.dist((self.x, self.y), self.current_target) < 0.3:
            self.inventory.pop(0); self.target_index, self.current_target = self.saved_patrol_index, self.saved_patrol_target; self.state = 'SEARCHING'
        elif self.state == 'RETURNING_HOME' and math.dist((self.x, self.y), (0.5, 0.5)) < 0.2: self.linear_velocity,self.angular_velocity,self.state=0.0,0.0,'CHARGING'
        elif self.state == 'CHARGING':
            self.battery += CONFIG["charge_rate"] * CONFIG["time_step"]
            if self.battery >= CONFIG["battery_capacity"]: self.battery=CONFIG["battery_capacity"]; self.state,self.current_target = self.saved_state,self.saved_target
        elif self.state == 'FINISHED': self.linear_velocity, self.angular_velocity = 0.0, 0.0

# --- UI Callbacks ---
def toggle_simulation(b):
    global simulation_running;
    if b.text in ["Start Simulation", "Reset"]: reset_simulation(); simulation_running=True; b.text="Pause"
    else: simulation_running=not simulation_running; b.text="Pause" if simulation_running else "Resume"
def set_num_gnomes(w):
    if w.number and w.number > 0: CONFIG["num_gnomes"]=int(w.number)
def set_robot_speed(s):
    CONFIG["robot_speed"]=s.value; wtext_speed.text=f'Robot Speed: {s.value:.1f} m/s'
def set_fov(s):
    CONFIG["camera_fov_degrees"]=s.value; wtext_fov.text=f'Camera FOV: {s.value:.0f}°';
    if robot: robot.camera_fov_rad=math.radians(s.value)
def set_low_battery_threshold(s):
    CONFIG["battery_low_threshold"]=s.value; wtext_low_battery.text=f'Low Battery Threshold: {s.value:.0f}%'
def set_drain_rate(s):
    CONFIG["drain_move"]=s.value; wtext_drain_rate.text=f'Move Drain Rate: {s.value:.1f}/s'
def set_charge_rate(s):
    CONFIG["charge_rate"]=s.value; wtext_charge_rate.text=f'Charge Rate: {s.value:.1f}/s'

def reset_simulation():
    global world, robot, vpython_objects, discovered_gnome_count
    for k, o_list in list(vpython_objects.items()):
        for o in o_list: o.visible=False; del o;
        del vpython_objects[k]
    world, robot = World(), SimulatedRobot()
    discovered_gnome_count = 0 # <<< MODIFIED: Reset counter
    vpython_objects["gnomes"] = [cylinder(pos=vector(p[0],0.2,p[1]),axis=vector(0,0.4,0),radius=0.15,color=color.red) for p in world.gnomes]
    r_obj=compound([cylinder(axis=vector(0,0.3,0),radius=0.25,color=color.blue), arrow(pos=vector(0,0.15,0),axis=vector(0.3,0,0),color=color.white,shaftwidth=0.05)])
    fov=cone(angle=math.radians(CONFIG["camera_fov_degrees"]),length=CONFIG["camera_range"],color=color.yellow,opacity=0.3)
    vpython_objects["robot"] = [r_obj, fov]
    
    # <<< MODIFIED: Clear the 2D map at the start
    gnome_dots.data, robot_path_curve.data = [], []
    gnome_list_display.text = "- Ningún gnomo detectado aún -"
    
    label_status.text = "Listo para buscar..."

# --- VPython Scene and UI Setup ---
scene.width, scene.height = 800, 600
scene.background, scene.center = color.cyan, vector(5, 0, 5)

map_display = graph(width=400, height=400, title="<b>2D Mission Map</b>", xtitle="X-axis", ytitle="Z-axis", xmin=0, xmax=10, ymin=0, ymax=10, fast=False, append_to_element="#right-panel")
for i in range(11): gcurve(graph=map_display,pos=[(i,0),(i,10)],color=color.gray(0.8)); gcurve(graph=map_display,pos=[(0,i),(10,i)],color=color.gray(0.8))
gnome_dots, robot_path_curve = gdots(graph=map_display,color=color.red,radius=4), gcurve(graph=map_display,color=color.blue,width=2)

box(pos=vector(5,-0.1,5),size=vector(10,0.2,10),color=color.green)
cylinder(pos=vector(0.5,0,0.5),axis=vector(0,0.1,0),radius=0.5,color=color.white,opacity=0.5)
box(pos=vector(5,5,5),size=vector(4,0.4,0.2),color=color.gray(0.5))
battery_bar=box(pos=vector(3,5,5),size=vector(4,0.4,0.2),color=color.green)
label_status=label(pos=vector(5,4.5,5),text="Press 'Start Simulation'",box=False,height=15)
scene.append_to_caption('\n<b>Simulation Controls</b>\n'); run_button=button(text="Start Simulation",bind=toggle_simulation)
scene.append_to_caption('  '); winput(prompt="Gnomes: ",type="numeric",text=CONFIG["num_gnomes"],bind=set_num_gnomes)
scene.append_to_caption('\n\n'); wtext_speed=wtext(text=f'Robot Speed: {CONFIG["robot_speed"]:.1f} m/s'); slider(min=0.5,max=5.0,value=CONFIG["robot_speed"],step=0.1,bind=set_robot_speed)
scene.append_to_caption('\n'); wtext_fov=wtext(text=f'Camera FOV: {CONFIG["camera_fov_degrees"]:.0f}°'); slider(min=10,max=120,value=CONFIG["camera_fov_degrees"],step=1,bind=set_fov)
scene.append_to_caption('\n\n<b>Parameters</b>\n')
wtext_low_battery = wtext(text=f'Low Battery Threshold: {CONFIG["battery_low_threshold"]:.0f}%'); slider(min=10, max=50, value=CONFIG["battery_low_threshold"], step=1, bind=set_low_battery_threshold)
scene.append_to_caption('\n'); wtext_drain_rate = wtext(text=f'Move Drain Rate: {CONFIG["drain_move"]:.1f}/s'); slider(min=0.1, max=5.0, value=CONFIG["drain_move"], step=0.1, bind=set_drain_rate)
scene.append_to_caption('\n'); wtext_charge_rate = wtext(text=f'Charge Rate: {CONFIG["charge_rate"]:.1f}/s'); slider(min=1, max=30, value=CONFIG["charge_rate"], step=1, bind=set_charge_rate)
scene.append_to_caption('\n\n<b>Gnome Positions</b>\n'); gnome_list_display = wtext(text="- Simulation not started -", append_to_element="#right-panel")

# --- Main Loop ---
while True:
    rate(100) 
    if not simulation_running or robot is None: continue
    
    gnomes_before = set(world.gnomes); robot.update_logic(world); robot.update_physics_and_battery(CONFIG["time_step"]); gnomes_after = set(world.gnomes)
    robot_path_curve.plot(pos=(robot.x, robot.y))
    
    # <<< NEW: Real-time map update logic
    if robot.newly_discovered_gnomes:
        if "- Ningún gnomo detectado aún -" in gnome_list_display.text:
            gnome_list_display.text = ""
        for new_gnome_pos in robot.newly_discovered_gnomes:
            discovered_gnome_count += 1
            gnome_dots.plot(new_gnome_pos)
            gnome_list_display.text += f"Descubierto {discovered_gnome_count}: ({new_gnome_pos[0]:.1f}, {new_gnome_pos[1]:.1f})\n"
        robot.newly_discovered_gnomes.clear()

    if "robot" in vpython_objects:
        r_obj, fov_cone = vpython_objects["robot"]; r_pos = vector(robot.x, 0, robot.y); r_axis = vector(math.cos(robot.theta), 0, math.sin(robot.theta))
        r_obj.pos, r_obj.axis = r_pos, r_axis
        cone_base_pos = r_pos + vector(0, 0.2, 0)
        fov_cone.pos, fov_cone.axis, fov_cone.angle = cone_base_pos + r_axis * robot.camera_range, -r_axis * robot.camera_range, robot.camera_fov_rad
        
    pct = robot.battery / CONFIG["battery_capacity"]; battery_bar.size.x, battery_bar.pos.x = 4*pct, 3+2*(1-pct)
    if pct < 0.25: battery_bar.color=color.red
    elif pct < 0.5: battery_bar.color=color.orange
    else: battery_bar.color=color.green
    label_status.text = f"State: {robot.state}\nBattery: {robot.battery:.1f}%"
    
    if gnomes_before-gnomes_after:
        rem_pos=next(iter(gnomes_before-gnomes_after)); rem_obj=None
        if "gnomes" in vpython_objects:
            for o in vpython_objects["gnomes"]:
                if o.pos.x==rem_pos[0] and o.pos.z==rem_pos[1]: rem_obj=o; break
            if rem_obj: rem_obj.visible=False; vpython_objects["gnomes"].remove(rem_obj); del rem_obj
            
    if robot.state == 'FINISHED' or robot.battery <= 0:
        simulation_running, run_button.text = False, "Reset"; label_status.text = "¡Batería agotada!" if robot.battery <= 0 else "¡Misión Completa!"