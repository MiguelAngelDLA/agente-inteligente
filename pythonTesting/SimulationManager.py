import math
from vpython import *

from World import World
from SimulatedRobot import SimulatedRobot

DEFAULT_CONFIG = {
    "num_gnomes": 15, "robot_speed": 1.5, "Kp_turn": 4.0, "camera_fov_degrees": 70,
    "battery_capacity": 100.0, "battery_low_threshold": 25.0, "time_step": 0.01,
    "camera_range": 3.0, "drain_base": 0.1, "drain_move": 1.0, "drain_turn": 0.5, "charge_rate": 10.0,
}

class SimulationManager:
    def __init__(self, config):
        self.config = config
        self.simulation_running = False
        self.world = None
        self.robot = None
        
        self.vpython_objects = {}
        self.vpython_widgets = {}

        self._setup_scene()
        self._setup_ui()
        self.reset_simulation()

    def _setup_scene(self):
        scene.width, scene.height = 800, 600
        scene.background, scene.center = color.cyan, vector(5, 0, 5)
        
        box(pos=vector(5, -0.1, 5), size=vector(10, 0.2, 10), color=color.green)
        cylinder(pos=vector(0.5, 0, 0.5), axis=vector(0, 0.1, 0), radius=0.5, color=color.white, opacity=0.5)
        
        # HUD de la batería
        box(pos=vector(5, 5, 5), size=vector(4, 0.4, 0.2), color=color.gray(0.5))
        self.vpython_objects["battery_bar"] = box(pos=vector(3, 5, 5), size=vector(4, 0.4, 0.2), color=color.green)
        self.vpython_objects["label_status"] = label(pos=vector(5, 4.5, 5), text="Presione 'Empezar simulación'", box=False, height=15)
        
        # Mapa 2D
        map_display = graph(width=400, height=400, title="<b>Mapa 2D</b>", xtitle="X-axis", ytitle="Z-axis", xmin=0, xmax=10, ymin=0, ymax=10, fast=False, append_to_element="#right-panel")
        for i in range(11):
            gcurve(graph=map_display, pos=[(i, 0), (i, 10)], color=color.gray(0.8))
            gcurve(graph=map_display, pos=[(0, i), (10, i)], color=color.gray(0.8))
        self.vpython_objects["gnome_dots"] = gdots(graph=map_display, color=color.red, radius=4)
        self.vpython_objects["robot_path_curve"] = gcurve(graph=map_display, color=color.blue, width=2)
    
    def _setup_ui(self):
        scene.append_to_caption('\n<b>Simulation Controls</b>\n')
        self.vpython_widgets["run_button"] = button(text="Empezar simulación", bind=self.toggle_simulation)
        scene.append_to_caption('  ')
        winput(prompt="Gnomes: ", type="numeric", text=self.config["num_gnomes"], bind=self.set_num_gnomes)

        scene.append_to_caption('\n\n')
        self.vpython_widgets["wtext_speed"] = wtext(text=f'Velocidad Robot: {self.config["robot_speed"]:.1f} m/s')
        slider(min=0.5, max=5.0, value=self.config["robot_speed"], step=0.1, bind=self.set_robot_speed)

        scene.append_to_caption('\n')
        self.vpython_widgets["wtext_fov"] = wtext(text=f'FOV Camara: {self.config["camera_fov_degrees"]:.0f}°')
        slider(min=10, max=120, value=self.config["camera_fov_degrees"], step=1, bind=self.set_fov)

        scene.append_to_caption('\n\n<b>Parametros</b>\n')
        self.vpython_widgets["wtext_low_battery"] = wtext(text=f'Límite de batería baja: {self.config["battery_low_threshold"]:.0f}%')
        slider(min=10, max=50, value=self.config["battery_low_threshold"], step=1, bind=self.set_low_battery_threshold)
        
        scene.append_to_caption('\n')
        self.vpython_widgets["wtext_drain_rate"] = wtext(text=f'Tasa de drenado de batería: {self.config["drain_move"]:.1f}/s')
        slider(min=0.1, max=5.0, value=self.config["drain_move"], step=0.1, bind=self.set_drain_rate)
        
        scene.append_to_caption('\n')
        self.vpython_widgets["wtext_charge_rate"] = wtext(text=f'Tasa de carga: {self.config["charge_rate"]:.1f}/s')
        slider(min=1, max=30, value=self.config["charge_rate"], step=1, bind=self.set_charge_rate)

        scene.append_to_caption('\n\n<b>Posiciones Gnomos</b>\n')
        self.vpython_widgets["gnome_list"] = wtext(text="- Simulación no empezada -", append_to_element="#right-panel")

    def reset_simulation(self):
        for key in ["gnomes", "robot"]:
            if key in self.vpython_objects:
                for obj in self.vpython_objects[key]:
                    obj.visible = False
                    del obj
                self.vpython_objects[key] = []
        
        self.world = World(self.config)
        self.robot = SimulatedRobot(self.config)
        self.discovered_gnome_count = 0

        self.vpython_objects["gnomes"] = [
            cylinder(pos=vector(p[0], 0.2, p[1]), axis=vector(0, 0.4, 0), radius=0.15, color=color.red)
            for p in self.world.gnomes
        ]
        r_body = compound([
            cylinder(axis=vector(0, 0.3, 0), radius=0.25, color=color.blue),
            arrow(pos=vector(0, 0.15, 0), axis=vector(0.3, 0, 0), color=color.white, shaftwidth=0.05)
        ])
        fov_cone = cone(angle=self.robot.camera_fov_rad, length=self.robot.camera_range, color=color.yellow, opacity=0.3)
        self.vpython_objects["robot"] = [r_body, fov_cone]
        
        self.vpython_objects["gnome_dots"].data = []
        self.vpython_objects["robot_path_curve"].data = []
        self.vpython_widgets["gnome_list"].text = "- Ningún gnomo detectado aún -"
        self.vpython_objects["label_status"].text = "Listo para buscar..."
        self.update_visuals()

    def update_visuals(self):
        if not self.robot: return

        r_obj, fov_cone = self.vpython_objects["robot"]
        r_pos = vector(self.robot.x, 0, self.robot.y)
        r_axis = vector(math.cos(self.robot.theta), 0, math.sin(self.robot.theta))
        r_obj.pos, r_obj.axis = r_pos, r_axis
    
        cone_tip_pos = r_pos + vector(0, 0.2, 0)

        cone_axis = r_axis * self.robot.camera_range

        fov_cone.pos = cone_tip_pos
        fov_cone.axis = cone_axis
        fov_cone.angle = self.robot.camera_fov_rad
    
        pct = self.robot.battery / self.config["battery_capacity"]
        self.vpython_objects["battery_bar"].size.x = 4 * pct
        self.vpython_objects["battery_bar"].pos.x = 3 + 2 * (1 - pct)
        if pct < 0.25: self.vpython_objects["battery_bar"].color = color.red
        elif pct < 0.5: self.vpython_objects["battery_bar"].color = color.orange
        else: self.vpython_objects["battery_bar"].color = color.green
        self.vpython_objects["label_status"].text = f"Estado: {self.robot.state}\nBatería: {self.robot.battery:.1f}%"


    def _update_map_and_lists(self):
        self.vpython_objects["robot_path_curve"].plot(pos=(self.robot.x, self.robot.y))
        
        if self.robot.newly_discovered_gnomes:
            if "- Ningún gnomo detectado aún -" in self.vpython_widgets["gnome_list"].text:
                self.vpython_widgets["gnome_list"].text = ""
            for new_gnome_pos in self.robot.newly_discovered_gnomes:
                self.discovered_gnome_count += 1
                self.vpython_objects["gnome_dots"].plot(new_gnome_pos)
                self.vpython_widgets["gnome_list"].text += f"Descubierto {self.discovered_gnome_count}: ({new_gnome_pos[0]:.1f}, {new_gnome_pos[1]:.1f})\n"
            self.robot.newly_discovered_gnomes.clear()

    def run(self):
        while True:
            rate(100)
            if not self.simulation_running:
                continue

            gnomes_before = set(self.world.gnomes)
            self.robot.update(self.world, self.config["time_step"])
            gnomes_after = set(self.world.gnomes)

            self.update_visuals()
            self._update_map_and_lists()
            
            removed_gnomes = gnomes_before - gnomes_after
            if removed_gnomes:
                rem_pos = removed_gnomes.pop()
                rem_obj = None
                for obj in self.vpython_objects["gnomes"]:
                    if obj.pos.x == rem_pos[0] and obj.pos.z == rem_pos[1]:
                        rem_obj = obj; break
                if rem_obj:
                    rem_obj.visible = False
                    self.vpython_objects["gnomes"].remove(rem_obj)
                    del rem_obj

            if self.robot.state == 'FINISHED' or self.robot.battery <= 0:
                self.simulation_running = False
                self.vpython_widgets["run_button"].text = "Reiniciar"
                final_text = "¡Batería agotada!" if self.robot.battery <= 0 else "¡Misión Completa!"
                self.vpython_objects["label_status"].text = final_text

    # --- Callbacks de la UI ---
    def toggle_simulation(self, b):
        if b.text in ["Empezar simulación", "Reiniciar"]:
            self.reset_simulation()
            self.simulation_running = True
            b.text = "Pausa"
        else:
            self.simulation_running = not self.simulation_running
            b.text = "Pausa" if self.simulation_running else "Seguir"

    def set_num_gnomes(self, w):
        if w.number and w.number > 0: self.config["num_gnomes"] = int(w.number)
    
    def set_robot_speed(self, s):
        self.config["robot_speed"] = s.value
        self.vpython_widgets["wtext_speed"].text = f'Velocidad del robot: {s.value:.1f} m/s'
    
    def set_fov(self, s):
        self.config["camera_fov_degrees"] = s.value
        self.vpython_widgets["wtext_fov"].text = f'Camera FOV: {s.value:.0f}°'
        if self.robot: self.robot.camera_fov_rad = math.radians(s.value)
    
    def set_low_battery_threshold(self, s):
        self.config["battery_low_threshold"] = s.value
        self.vpython_widgets["wtext_low_battery"].text = f'Límite de batería baja: {s.value:.0f}%'
    
    def set_drain_rate(self, s):
        self.config["drain_move"] = s.value
        self.vpython_widgets["wtext_drain_rate"].text = f'Tasa de drenado de bateria: {s.value:.1f}/s'
    
    def set_charge_rate(self, s):
        self.config["charge_rate"] = s.value
        self.vpython_widgets["wtext_charge_rate"].text = f'Tasa de carga: {s.value:.1f}/s'


if __name__ == "__main__":
    simulation = SimulationManager(DEFAULT_CONFIG)
    simulation.run()