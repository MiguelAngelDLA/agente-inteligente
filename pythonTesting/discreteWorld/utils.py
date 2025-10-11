# utils.py
import pygame
import heapq
from constants import CONFIG, MEM_OBSTACLE, MEM_UNKNOWN, MEM_SLOW, COLOR_ROBOT, COLOR_TEXT

class Node:
    def __init__(self, position, parent=None):
        self.position, self.parent = position, parent
        self.g, self.h, self.f = 0, 0, 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f
    def __repr__(self): return f"Node({self.position})"
    pass

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


    pass

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
