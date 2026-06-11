"""
Path Planning Module
Advanced path planning algorithms for autonomous navigation
"""

import heapq
import math
from typing import List, Tuple, Dict, Optional
import logging

logger = logging.getLogger(__name__)

class Node:
    """Node for A* pathfinding"""
    def __init__(self, position: Tuple[int, int], g_cost: float = 0, h_cost: float = 0, parent=None):
        self.position = position
        self.g_cost = g_cost  # Cost from start to current node
        self.h_cost = h_cost  # Heuristic cost to goal
        self.f_cost = g_cost + h_cost  # Total cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class GridMap:
    """Grid-based map for path planning"""

    def __init__(self, width: int, height: int, resolution: float = 0.1):
        self.width = width
        self.height = height
        self.resolution = resolution  # meters per cell
        self.grid = [[0 for _ in range(width)] for _ in range(height)]  # 0 = free, 1 = obstacle
        self.obstacles = []

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int(world_x / self.resolution)
        grid_y = int(world_y / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.resolution
        world_y = grid_y * self.resolution
        return world_x, world_y

    def is_valid_position(self, x: int, y: int) -> bool:
        """Check if grid position is valid"""
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, x: int, y: int) -> bool:
        """Check if grid cell is free"""
        if not self.is_valid_position(x, y):
            return False
        return self.grid[y][x] == 0

    def set_obstacle(self, x: int, y: int):
        """Set grid cell as obstacle"""
        if self.is_valid_position(x, y):
            self.grid[y][x] = 1
            self.obstacles.append((x, y))

    def add_sensor_obstacles(self, sensor_data: Dict, robot_position: Tuple[float, float]):
        """Add obstacles from sensor data"""
        # Clear previous sensor obstacles (keep static obstacles)
        for obs in self.obstacles[:]:  # Copy list to avoid modification during iteration
            if hasattr(obs, 'sensor_based') and obs.sensor_based:
                x, y = obs.position
                self.grid[y][x] = 0
                self.obstacles.remove(obs)

        # Add new sensor-based obstacles
        ir_sensors = sensor_data.get('laser_sensors', {})
        ultrasonic = sensor_data.get('ultrasonic_sensors', {})

        # Process IR sensor data
        for sensor_name, distance in ir_sensors.items():
            if distance and distance > 0 and distance < 2000:  # Valid reading under 2m
                angle_offset = {
                    'left_front': -45, 'left_back': -135,
                    'right_front': 45, 'right_back': 135,
                    'back_left': -135, 'back_right': 135
                }.get(sensor_name, 0)

                # Calculate obstacle position
                obstacle_x = robot_position[0] + distance * math.cos(math.radians(angle_offset))
                obstacle_y = robot_position[1] + distance * math.sin(math.radians(angle_offset))

                grid_x, grid_y = self.world_to_grid(obstacle_x, obstacle_y)
                if self.is_valid_position(grid_x, grid_y):
                    self.set_obstacle(grid_x, grid_y)
                    # Mark as sensor-based for clearing
                    self.obstacles[-1] = type('Obstacle', (), {
                        'position': (grid_x, grid_y),
                        'sensor_based': True
                    })()

class PathPlanner:
    """Advanced path planning with A* algorithm"""

    def __init__(self, grid_map: GridMap):
        self.grid_map = grid_map
        self.directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinal directions
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal directions
        ]

    def heuristic(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic cost (Manhattan distance)"""
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def get_neighbors(self, position: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring positions"""
        neighbors = []
        for dx, dy in self.directions:
            new_x, new_y = position[0] + dx, position[1] + dy
            if self.grid_map.is_free(new_x, new_y):
                neighbors.append((new_x, new_y))
        return neighbors

    def reconstruct_path(self, current: Node) -> List[Tuple[int, int]]:
        """Reconstruct path from goal node to start"""
        path = []
        while current:
            path.append(current.position)
            current = current.parent
        path.reverse()
        return path

    def find_path(self, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Find path using A* algorithm"""
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
