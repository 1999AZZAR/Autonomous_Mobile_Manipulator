"""
Path Planning Module
Advanced path planning algorithms for autonomous navigation
"""

import heapq
import math
import random
from typing import List, Tuple, Dict, Optional
import logging

import numpy as np
from scipy.spatial import KDTree

logger = logging.getLogger(__name__)

class Node:
    """Node for A* pathfinding"""
    def __init__(self, position: Tuple[int, int], g_cost: float = 0, h_cost: float = 0, parent=None):
        self.position = position
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class GridMap:
    """Grid-based map for path planning"""

    def __init__(self, width: int, height: int, resolution: float = 0.1):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.obstacles = []
        self.sensor_obstacles: Dict[Tuple[int, int], int] = {}
        self._age_counter = 0

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        grid_x = int(world_x / self.resolution)
        grid_y = int(world_y / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        world_x = grid_x * self.resolution
        world_y = grid_y * self.resolution
        return world_x, world_y

    def is_valid_position(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, x: int, y: int) -> bool:
        if not self.is_valid_position(x, y):
            return False
        return self.grid[y][x] == 0

    def set_obstacle(self, x: int, y: int):
        if self.is_valid_position(x, y):
            self.grid[y][x] = 1
            self.obstacles.append((x, y))

    def add_sensor_obstacles(self, sensor_data: Dict, robot_position: Tuple[float, float], max_age: int = 5):
        """Project sensor readings onto grid with automatic aging.

        Clears obstacles not refreshed in *max_age* cycles, then adds current
        IR + ultrasonic readings.  Distances are converted from mm to m
        internally.
        """
        self._age_counter += 1

        # ---- 1. clear stale sensor obstacles ----
        stale = [(gx, gy) for (gx, gy), seen in self.sensor_obstacles.items()
                 if self._age_counter - seen >= max_age]
        for gx, gy in stale:
            del self.sensor_obstacles[(gx, gy)]
            if self.is_valid_position(gx, gy):
                self.grid[gy][gx] = 0

        # ---- 2. clear all currently-tracked sensor cells (will re-add below) ----
        for gx, gy in list(self.sensor_obstacles.keys()):
            if self.is_valid_position(gx, gy):
                self.grid[gy][gx] = 0
        self.sensor_obstacles.clear()

        rx, ry = robot_position

        # ---- 3. IR (laser) sensors ----
        ir = sensor_data.get('laser_sensors', {})
        for name, dist_mm in ir.items():
            if not dist_mm or dist_mm <= 0 or dist_mm >= 2000:
                continue
            angle = {
                'left_front': -45, 'left_back': -135,
                'right_front': 45, 'right_back': 135,
                'back_left': -135, 'back_right': 135,
            }.get(name, 0)
            dist_m = dist_mm / 1000.0
            ox = rx + dist_m * math.cos(math.radians(angle))
            oy = ry + dist_m * math.sin(math.radians(angle))
            gx, gy = self.world_to_grid(ox, oy)
            if self.is_valid_position(gx, gy):
                self.grid[gy][gx] = 1
                self.sensor_obstacles[(gx, gy)] = self._age_counter

        # ---- 4. ultrasonic sensors (wider footprint) ----
        us = sensor_data.get('ultrasonic_sensors', {})
        for name, dist_mm in us.items():
            if not dist_mm or dist_mm <= 0 or dist_mm >= 4000:
                continue
            # Ultrasonic has ~30° beam — spread obstacle across 3 cells
            base_angle = {'front_left': 30, 'front_right': -30}.get(name, 0)
            dist_m = dist_mm / 1000.0
            for spread in (-15, 0, 15):
                ox = rx + dist_m * math.cos(math.radians(base_angle + spread))
                oy = ry + dist_m * math.sin(math.radians(base_angle + spread))
                gx, gy = self.world_to_grid(ox, oy)
                if self.is_valid_position(gx, gy):
                    self.grid[gy][gx] = 1
                    self.sensor_obstacles[(gx, gy)] = self._age_counter

    def clear_static_obstacles(self):
        """Remove all non-sensor obstacles (for dynamic environments)."""
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.obstacles.clear()

    def get_obstacle_grid(self) -> List[List[int]]:
        """Return a copy of the current grid (for visualisation / frontend)."""
        return [row[:] for row in self.grid]

_RRT_WARNED = False  # module-level flag to avoid log spam

class RRTNode:
    """Node for RRT* pathfinding"""
    __slots__ = ('position', 'parent', 'cost')
    def __init__(self, position: Tuple[float, float], parent=None):
        self.position = position
        self.parent = parent
        self.cost = 0.0

class RRTStarPlanner:
    """Optimal Rapidly-exploring Random Tree (RRT*) path planner.

    Uses scipy.spatial.KDTree for O(log n) nearest-neighbour queries and
    O(log n + k) range searches instead of the original O(n) linear scans.
    """

    def __init__(self, grid_map: GridMap, max_iter: int = 1000, step_size: float = 0.5, search_radius: float = 1.0):
        self.grid_map = grid_map
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.nodes: List[RRTNode] = []
        self._positions: List[Tuple[float, float]] = []  # mirror of node positions for KD-tree

    def get_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def is_collision_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        dist = self.get_distance(p1, p2)
        if dist == 0:
            return True
        res = self.grid_map.resolution / 2
        steps = int(dist / res)
        if steps == 0:
            return self.grid_map.is_free(*self.grid_map.world_to_grid(p2[0], p2[1]))
        for i in range(steps + 1):
            t = i / steps
            curr_x = p1[0] + (p2[0] - p1[0]) * t
            curr_y = p1[1] + (p2[1] - p1[1]) * t
            if not self.grid_map.is_free(*self.grid_map.world_to_grid(curr_x, curr_y)):
                return False
        return True

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        self.nodes = [RRTNode(start)]
        self._positions = [(start[0], start[1])]
        goal_node = None

        for _ in range(self.max_iter):
            if random.random() < 0.1:
                rand_pos = goal
            else:
                rand_pos = (
                    random.uniform(0, self.grid_map.width * self.grid_map.resolution),
                    random.uniform(0, self.grid_map.height * self.grid_map.resolution),
                )

            # ---- nearest neighbour via KD-tree ----
            try:
                kdt = KDTree(self._positions)
                _, idx = kdt.query([rand_pos], k=1)
                nearest_node = self.nodes[idx[0][0]]
            except Exception:
                # Fallback if KD-tree fails (e.g., duplicate points)
                nearest_node = min(self.nodes, key=lambda n: self.get_distance(n.position, rand_pos))

            # ---- step ----
            dist = self.get_distance(nearest_node.position, rand_pos)
            if dist > self.step_size:
                theta = math.atan2(rand_pos[1] - nearest_node.position[1],
                                   rand_pos[0] - nearest_node.position[0])
                new_pos = (nearest_node.position[0] + self.step_size * math.cos(theta),
                           nearest_node.position[1] + self.step_size * math.sin(theta))
            else:
                new_pos = rand_pos

            if not self.is_collision_free(nearest_node.position, new_pos):
                continue

            new_node = RRTNode(new_pos, nearest_node)
            new_node.cost = nearest_node.cost + self.get_distance(nearest_node.position, new_pos)

            # ---- neighbours via KD-tree range search ----
            try:
                idx_nb = kdt.query_ball_point(new_pos, self.search_radius)
                neighbors = [self.nodes[i] for i in idx_nb if i < len(self.nodes)]
            except Exception:
                neighbors = [n for n in self.nodes
                             if self.get_distance(n.position, new_pos) < self.search_radius]

            for nb in neighbors:
                c = nb.cost + self.get_distance(nb.position, new_pos)
                if c < new_node.cost and self.is_collision_free(nb.position, new_pos):
                    new_node.parent = nb
                    new_node.cost = c

            self.nodes.append(new_node)
            self._positions.append(new_pos)

            # ---- rewire neighbours ----
            for nb in neighbors:
                c = new_node.cost + self.get_distance(new_node.position, nb.position)
                if c < nb.cost and self.is_collision_free(new_node.position, nb.position):
                    nb.parent = new_node
                    nb.cost = c

            # ---- check goal ----
            if self.get_distance(new_pos, goal) < self.step_size:
                if self.is_collision_free(new_pos, goal):
                    if goal_node is None or new_node.cost + self.get_distance(new_pos, goal) < goal_node.cost:
                        goal_node = RRTNode(goal, new_node)
                        goal_node.cost = new_node.cost + self.get_distance(new_pos, goal)

        if goal_node is None:
            return None

        path = []
        curr = goal_node
        while curr:
            path.append(curr.position)
            curr = curr.parent
        path.reverse()
        return path

class PathPlanner:
    """Advanced path planning with multiple algorithms"""

    def __init__(self, grid_map: GridMap, algorithm: str = 'astar'):
        self.grid_map = grid_map
        self.algorithm = algorithm
        self.astar_planner = None # Initialized on demand
        self.rrt_planner = RRTStarPlanner(grid_map)
        self.directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinal directions
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal directions
        ]

    def set_algorithm(self, algorithm: str):
        self.algorithm = algorithm

    def find_path(self, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        if self.algorithm == 'rrtstar':
            return self.rrt_planner.find_path(start_world, goal_world)
        
        # Default to A*
        return self._find_path_astar(start_world, goal_world)

    def _find_path_astar(self, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        # Convert to grid coordinates
        start_grid = self.grid_map.world_to_grid(start_world[0], start_world[1])
        goal_grid = self.grid_map.world_to_grid(goal_world[0], goal_world[1])

        # Validate positions
        if not self.grid_map.is_free(*start_grid) or not self.grid_map.is_free(*goal_grid):
            logger.warning("Start or goal position is not free")
            return None

        # A* algorithm
        open_set = []
        closed_set = set()

        start_node = Node(start_grid, 0, self.heuristic(start_grid, goal_grid))
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            if current.position == goal_grid:
                # Path found
                grid_path = self.reconstruct_path(current)
                # Convert back to world coordinates
                world_path = [self.grid_map.grid_to_world(x, y) for x, y in grid_path]
                return world_path

            closed_set.add(current.position)

            for neighbor_pos in self.get_neighbors(current.position):
                if neighbor_pos in closed_set:
                    continue

                g_cost = current.g_cost + 1  # Assume cost of 1 for each step
                h_cost = self.heuristic(neighbor_pos, goal_grid)
                neighbor_node = Node(neighbor_pos, g_cost, h_cost, current)

                # Check if this path is better
                better_path = True
                for open_node in open_set:
                    if neighbor_pos == open_node.position and g_cost >= open_node.g_cost:
                        better_path = False
                        break

                if better_path:
                    heapq.heappush(open_set, neighbor_node)

        # No path found
        logger.warning("No path found to goal")
        return None

class MovementSequence:
    """Movement sequence planning and optimization"""

    def __init__(self):
        # Command library - matches Arduino firmware commands
        # Movement: 8 base movements (f, b, q, e, z, x, t, y) + stop (s)
        self.command_library = {
            'f': {'type': 'move', 'direction': (1, 0), 'duration': 1.0},   # Forward: FR+FL
            'b': {'type': 'move', 'direction': (-1, 0), 'duration': 1.0},  # Backward: FR+FL
            'q': {'type': 'move', 'direction': (0.7, 0.7), 'duration': 1.0},  # Forward-Left: FR+Back
            'e': {'type': 'move', 'direction': (0.7, -0.7), 'duration': 1.0},  # Forward-Right: FL+Back
            'z': {'type': 'move', 'direction': (-0.7, 0.7), 'duration': 1.0},  # Backward-Left: FR+Back reversed
            'x': {'type': 'move', 'direction': (-0.7, -0.7), 'duration': 1.0},  # Backward-Right: FL+Back reversed
            's': {'type': 'stop', 'duration': 0.1},                        # Stop
            't': {'type': 'rotate', 'direction': 1, 'duration': 2.0},      # Turn left (CCW): all 3
            'y': {'type': 'rotate', 'direction': -1, 'duration': 2.0},     # Turn right (CW): all 3
        }

    def optimize_sequence(self, commands: List[str]) -> List[str]:
        """Optimize movement sequence by removing redundant commands"""
        optimized = []
        i = 0

        while i < len(commands):
            command = commands[i]

            # Skip consecutive stops (keep only the last one)
            if command == 's':
                while i + 1 < len(commands) and commands[i + 1] == 's':
                    i += 1
                optimized.append('s')
            # Skip opposite direction commands that cancel each other
            elif command in ['f', 'b', 'q', 'e', 'z', 'x', 't', 'y'] and len(optimized) > 0:
                last_cmd = optimized[-1]
                opposites = {'f': 'b', 'b': 'f', 'q': 'x', 'x': 'q', 'e': 'z', 'z': 'e', 't': 'y', 'y': 't'}

                if last_cmd == opposites.get(command):
                    optimized.pop()
                else:
                    optimized.append(command)
            else:
                optimized.append(command)

            i += 1

        return optimized

    def estimate_duration(self, commands: List[str]) -> float:
        """Estimate total duration of command sequence"""
        total_time = 0.0

        for cmd in commands:
            if cmd in self.command_library:
                total_time += self.command_library[cmd]['duration']
            else:
                total_time += 0.5  # Default duration for unknown commands

        return total_time

    def generate_lawnmower_pattern(self, width: int, height: int, spacing: float = 1.0) -> List[str]:
        """Generate lawn mower pattern commands"""
        commands = []
        direction = 1  # 1 = right, -1 = left

        for row in range(height):
            # Move across the row
            for _ in range(width - 1):
                commands.append('f')

            # Turn around for next row (if not last row)
            if row < height - 1:
                commands.append('y')  # Turn right CW
                commands.append('f')  # Move to next row
                commands.append('y')  # Turn right again to face correct direction
                direction *= -1

            # Change direction for next row
            if direction == 1:
                commands.append('y')
            else:
                commands.append('t')

        commands.append('s')  # Stop at end
        return self.optimize_sequence(commands)

class WaypointNavigator:
    """Waypoint-based navigation system"""

    def __init__(self, path_planner: PathPlanner):
        self.path_planner = path_planner
        self.current_position = (0.0, 0.0)  # ENU coordinates
        self.waypoints = []

    def set_position(self, position: Tuple[float, float]):
        """Set current robot position"""
        self.current_position = position

    def add_waypoint(self, waypoint: Tuple[float, float]):
        """Add waypoint to navigation queue"""
        self.waypoints.append(waypoint)

    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()

    def navigate_to_next_waypoint(self) -> Optional[List[str]]:
        """Navigate to next waypoint in queue"""
        if not self.waypoints:
            return None

        next_waypoint = self.waypoints[0]

        # Find path to waypoint
        path = self.path_planner.find_path(self.current_position, next_waypoint)

        if not path or len(path) < 2:
            return None

        # Convert path to movement commands
        commands = self.path_to_commands(path)

        # Remove reached waypoint
        self.waypoints.pop(0)
        self.current_position = next_waypoint

        return commands

    def path_to_commands(self, path: List[Tuple[float, float]]) -> List[str]:
        """Convert path coordinates to movement commands"""
        commands = []

        for i in range(1, len(path)):
            current = path[i - 1]
            next_pos = path[i]

            # Calculate movement direction
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]

            # Determine primary direction using diagonal commands
            if abs(dx) > abs(dy):
                # Horizontal movement - use diagonal commands
                if dx > 0:
                    command = 'e'  # Forward-Right diagonal
                else:
                    command = 'q'  # Forward-Left diagonal
            else:
                # Vertical movement - use forward/backward
                command = 'f' if dy > 0 else 'b'

            commands.append(command)

        commands.append('s')  # Stop at destination
        return commands
