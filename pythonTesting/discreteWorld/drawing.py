# drawing.py
import pygame
import math
from constants import (CONFIG, CELL_SIZE, UI_WIDTH, COLOR_BG, COLOR_GRID, 
                       COLOR_OBSTACLE, COLOR_GNOME, COLOR_HOME, COLOR_SLOW_TERRAIN, 
                       COLOR_PATH, COLOR_VISION, COLOR_ROBOT, COLOR_TEXT, 
                       COLOR_BATTERY_GREEN, COLOR_BATTERY_ORANGE, COLOR_BATTERY_RED,
                       COLOR_TEXT_HIGHLIGHT)

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
