"""Planning and tracking regression tests for occupied grids and corners."""

import heapq
import math

import numpy as np
import pytest

from my_bot.astar import (
    GridGeometry, inflate_obstacles, pursuit_command, run_astar, segment_is_free,
)


def test_rotated_origin_and_negative_boundary():
    geometry = GridGeometry(0.05, 20, 15, -2.95, -2.59, math.pi / 2.0)
    for cell in ((0, 0), (4, 10), (14, 19)):
        assert geometry.world_to_grid(*geometry.grid_to_world(cell)) == cell
    unrotated = GridGeometry(1.0, 5, 5, 0.0, 0.0)
    assert unrotated.world_to_grid(-0.001, 0.0) == (0, -1)
    assert not unrotated.contains(unrotated.world_to_grid(5.0, 0.0))


def test_unknown_and_footprint_clearance():
    grid = np.zeros((11, 11), dtype=int)
    grid[5, 5] = 100
    grid[8, 8] = -1
    inflated = inflate_obstacles(grid, resolution=0.1, radius=0.21)
    assert inflated[5, 3] and inflated[5, 7]
    assert inflated[8, 8] and inflated[8, 7]
    assert inflated[0, 4]  # Unknown space beyond the map boundary.
    assert not inflated[5, 2]


@pytest.mark.parametrize('grid,start,end', [
    (np.array([[0, 100], [100, 0]]), (0, 0), (1, 1)),
    (np.array([[0, -1, 0]]), (0, 0), (0, 2)),
    (np.zeros((3, 3)), (-1, 0), (2, 2)),
    (np.array([[100, 0], [0, 0]]), (0, 0), (1, 1)),
])
def test_reject_unreachable_or_invalid_endpoints(grid, start, end):
    assert run_astar(grid, start, end) is None


def test_astar_cost_matches_dijkstra_on_maze():
    grid = np.zeros((18, 23), dtype=int)
    grid[:14, 5] = 100
    grid[4:, 10] = 100
    grid[:13, 16] = 100
    start, end = (1, 1), (16, 21)
    path = run_astar(grid, start, end)
    assert path is not None
    # Independent weighted uniform-cost search is the optimality reference.
    frontier, seen = [(0.0, start)], set()
    expected = None
    while frontier:
        cost, (y, x) = heapq.heappop(frontier)
        if (y, x) in seen:
            continue
        seen.add((y, x))
        if (y, x) == end:
            expected = cost
            break
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                ny, nx = y + dy, x + dx
                if not (dy or dx) or not (0 <= ny < 18 and 0 <= nx < 23):
                    continue
                if grid[ny, nx] or (dy and dx and (grid[y, nx] or grid[ny, x])):
                    continue
                heapq.heappush(frontier, (cost + math.hypot(dy, dx), (ny, nx)))
    actual = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    assert actual == pytest.approx(expected)
    assert path[0] == start and path[-1] == end


def test_tracking_cannot_cut_a_blocked_corner():
    geometry = GridGeometry(1.0, 3, 3, 0.0, 0.0)
    blocked = np.zeros((3, 3), dtype=bool)
    blocked[0, 1] = True
    assert not segment_is_free(geometry, blocked, (0.5, 0.5), (1.5, 1.5))
    assert segment_is_free(geometry, blocked, (0.5, 0.5), (0.5, 2.5))


def test_controller_rotates_for_target_behind_and_handles_zero_distance():
    linear, angular = pursuit_command((0.0, 0.0), 0.0, (-1.0, 0.0), 0.12, 0.6)
    assert linear == 0.0
    assert abs(angular) == pytest.approx(0.6)
    assert pursuit_command((0.0, 0.0), 0.0, (0.0, 0.0), 0.12, 0.6) == (0.0, 0.0)
    assert run_astar(np.zeros((2, 2)), (0, 0), (0, 0)) == [(0, 0)]
