# simulation.py
import pygame
from constants import CONFIG, CELL_SIZE, UI_WIDTH
from world import World, SimulatedRobot
from utils import Slider
import drawing

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
        total_width = screen_width + UI_WIDTH

        self.screen = pygame.display.set_mode((total_width, screen_height))
        pygame.display.set_caption("Simulación de Robot (Visor en Web)")

        self.world = World()
        self.robot = SimulatedRobot()
        
        self.sliders = [
            Slider(screen_width + 25, 520, UI_WIDTH - 50, 12, 5, 30, CONFIG['GRID_WIDTH'], "Ancho Mundo", 'GRID_WIDTH'),
            Slider(screen_width + 25, 560, UI_WIDTH - 50, 12, 5, 30, CONFIG['GRID_HEIGHT'], "Alto Mundo", 'GRID_HEIGHT'),
            Slider(screen_width + 25, 600, UI_WIDTH - 50, 12, 1.0, 10.0, CONFIG['animation_speed'], "Velocidad Anim.", 'animation_speed'),
        ]

    def run(self):
        running = True
        while running:
            dt_sec = self.clock.tick(60) / 1000.0
            
            # --- MANEJO DE EVENTOS ---
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

            # --- LÓGICA DE LA SIMULACIÓN ---
            if self.simulation_running:
                self.robot.update(self.world, dt_sec)
                if self.robot.state == 'TERMINADO' or self.robot.battery <= 0:
                    self.simulation_running = False

            # --- DIBUJADO ---
            self.screen.fill(drawing.COLOR_BG)
            drawing.draw_world(self.screen, self.world)
            drawing.draw_grid(self.screen, self.applied_grid_width * CELL_SIZE, self.applied_grid_height * CELL_SIZE)
            drawing.draw_path(self.screen, self.robot)
            drawing.draw_vision_cone(self.screen, self.robot)
            drawing.draw_robot(self.screen, self.robot)
            drawing.draw_ui(self.screen, self.robot, self.font, self.small_font, self.sliders, self.applied_grid_width, self.applied_grid_height)

            pygame.display.flip()
        
        pygame.quit()