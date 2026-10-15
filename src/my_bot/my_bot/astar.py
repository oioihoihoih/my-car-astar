"""Grid coordinates, footprint inflation, and optimal eight-neighbour A*."""

from dataclasses import dataclass
import heapq
import math

import numpy as np


def normalize_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    """Extract planar yaw from a ROS quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass(frozen=True)
class GridGeometry:
    """Convert world (x, y) to array (row, column), including rotated origins."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def world_to_grid(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((-s * dx + c * dy) / self.resolution),
                math.floor((c * dx + s * dy) / self.resolution))

    def grid_to_world(self, cell):
        y, x = cell
        dx, dy = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c * dx - s * dy,
                self.origin_y + s * dx + c * dy)

    def contains(self, cell):
        y, x = cell
        return 0 <= y < self.height and 0 <= x < self.width


def inflate_obstacles(map_data, resolution, radius, occupied_threshold=50):
    """Block unknown cells, obstacles, and their circular footprint clearance."""
    blocked = (map_data < 0) | (map_data >= occupied_threshold)
    cells = math.ceil(radius / resolution)
    if cells == 0:
        return blocked.copy()
    # Outside the map is blocked too; the footprint cannot cross its boundary.
    padded = np.pad(blocked, cells, constant_values=True)
    height, width = blocked.shape
    inflated = blocked.copy()
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if math.hypot(dx, dy) * resolution <= radius + 1e-9:
                inflated |= padded[cells + dy:cells + dy + height,
                                   cells + dx:cells + dx + width]
    return inflated


def run_astar(map_data, start, end, width=None, height=None):
    """Return (row, column) cells; zero is traversable, other values blocked.

    Diagonal steps cost sqrt(2); octile distance is an admissible heuristic.
    Both adjacent cardinal cells must be free before crossing a diagonal.
    """
    grid = np.asarray(map_data)
    if grid.ndim != 2:
        raise ValueError('A* needs a two-dimensional grid')
    rows, cols = grid.shape
    if (width is not None and width != cols) or (
            height is not None and height != rows):
        raise ValueError('Grid shape does not match width/height')
    for y, x in (start, end):
        if not (0 <= y < rows and 0 <= x < cols) or grid[y, x] != 0:
            return None

    def heuristic(cell):
        dy, dx = abs(cell[0] - end[0]), abs(cell[1] - end[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    queue = [(heuristic(start), 0.0, start)]
    scores, parents = {start: 0.0}, {}
    moves = ((0, 1), (0, -1), (1, 0), (-1, 0),
             (1, 1), (1, -1), (-1, 1), (-1, -1))
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost > scores[cell]:
            continue  # A better route to this cell was already queued.
        if cell == end:
            path = [cell]
            while cell in parents:
                cell = parents[cell]
                path.append(cell)
            return path[::-1]
        y, x = cell
        for dy, dx in moves:
            ny, nx = y + dy, x + dx
            if not (0 <= ny < rows and 0 <= nx < cols) or grid[ny, nx] != 0:
                continue
            if dy and dx and (grid[y, nx] != 0 or grid[ny, x] != 0):
                continue
            next_cell = (ny, nx)
            next_cost = cost + (math.sqrt(2.0) if dy and dx else 1.0)
            if next_cost < scores.get(next_cell, math.inf):
                scores[next_cell] = next_cost
                parents[next_cell] = cell
                heapq.heappush(queue, (next_cost + heuristic(next_cell),
                                      next_cost, next_cell))
    return None


def segment_is_free(geometry, blocked, start, end):
    """Check a tracking shortcut and prevent diagonal corner cutting."""
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, math.ceil(distance / (geometry.resolution * 0.25)))
    previous = geometry.world_to_grid(*start)
    for i in range(steps + 1):
        fraction = i / steps
        cell = geometry.world_to_grid(
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]))
        if not geometry.contains(cell) or blocked[cell]:
            return False
        if cell[0] != previous[0] and cell[1] != previous[1]:
            if blocked[previous[0], cell[1]] or blocked[cell[0], previous[1]]:
                return False
        previous = cell
    return True


def pursuit_command(position, yaw, target, speed, max_angular_speed,
                    rotate_threshold=0.7):
    """Return forward and angular speed, rotating first for sharp turns."""
    dx, dy = target[0] - position[0], target[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return 0.0, 0.0
    alpha = normalize_angle(math.atan2(dy, dx) - yaw)
    if abs(alpha) > rotate_threshold:
        return 0.0, max(-max_angular_speed, min(max_angular_speed, 1.5 * alpha))
    linear = speed * max(0.2, math.cos(alpha))
    angular = 2.0 * linear * math.sin(alpha) / distance
    return linear, max(-max_angular_speed, min(max_angular_speed, angular))
