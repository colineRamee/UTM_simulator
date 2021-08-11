"""
Simple static path planning utility
Graph: 8-connected grid, made of cells all initialized at the beginning
Shortest path finder: A*
Smoother: smoothes the path, with a collision check to ensure the path is still valid
"""

import numpy as np
import heapq
import math
from utm_simulator import my_utils


class Grid:
    """8-connected grid for path planning"""

    def __init__(self, max_grid_size, area_side_length, static_obstacles):
        self.n_grid = math.ceil(area_side_length / max_grid_size)
        self.cell_length = area_side_length / self.n_grid
        self.n_grid += 1
        self.cells = self.initialize_grid()
        self.static_obstacles = static_obstacles

    def initialize_grid(self):
        cells = np.empty((self.n_grid, self.n_grid), dtype='object')
        for i in range(0, self.n_grid):
            for j in range(0, self.n_grid):
                cells[i, j] = Cell(self.grid2cartesian(np.array([i, j])), [i, j])
        return cells

    def grid2cartesian(self, grid_coords):
        return grid_coords * self.cell_length

    def cartesian2grid(self, coords):
        return [int(round(coords[0] / self.cell_length)), int(round(coords[1] / self.cell_length))]

    def get_cell(self, coords=None, grid_coords=None):
        if grid_coords is not None:
            return self.cells[grid_coords[0], grid_coords[1]]
        else:
            i, j = self.cartesian2grid(coords)
            return self.cells[i, j]

    def get_neighbors(self, cell):
        i = int(cell.grid_coords[0])
        j = int(cell.grid_coords[1])
        i_index = [i]
        j_index = [j]
        if i > 0:
            i_index.append(i - 1)
        if j > 0:
            j_index.append(j - 1)
        if i < self.n_grid - 1:
            i_index.append(i + 1)
        if j < self.n_grid - 1:
            j_index.append(j + 1)
        neighbors = []
        for ii in i_index:
            for jj in j_index:
                if ii != i or jj != j:
                    neighbors.append(self.cells[ii, jj])
        return neighbors

    def get_heuristic_distance(self, origin_cell, destination_cell):
        d = 1
        d_diag = 1.4142135623730951 * d
        dx = abs(destination_cell.grid_coords[0] - origin_cell.grid_coords[0])
        dy = abs(destination_cell.grid_coords[1] - origin_cell.grid_coords[1])
        return d * (dx + dy) + (d_diag - 2 * d) * min(dx, dy)

    def get_travel_cost(self, cell_a, cell_b):
        return np.linalg.norm(cell_a.grid_coords - cell_b.grid_coords)

    def free_path(self, cell_a, cell_b):
        return self.check_collision(cell_a.coords, cell_b.coords)

    def check_collision(self, pos_a, pos_b):
        v = pos_b - pos_a
        v_squared = np.dot(v, v)
        for obstacle in self.static_obstacles:
            if v_squared != 0:
                t = - np.dot(v, pos_a - obstacle.position) / v_squared
                t = my_utils.clamp(0, 1, t)
            else:
                t = 0
            min_distance = np.linalg.norm(pos_a + t * v - obstacle.position)
            if min_distance < obstacle.radius:
                return False
        return True


class Cell:
    def __init__(self, coords, grid_coords):
        self.coords = np.array(coords, dtype='float')
        self.grid_coords = np.array(grid_coords, dtype='float')

    def __lt__(self, other):
        return self.coords[0] < other.coords[0]


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def push(self, state, priority, priority2):
        # not checking for duplicates. According to redblob games, actually faster than implementing a smarter heapq
        # to break the ties between two cells with same f, favors the one with lowest h
        heapq.heappush(self.elements, (priority, priority2, state))

    def pop(self):
        # return the lowest priority task
        while self.elements:
            priority, priority2, state = heapq.heappop(self.elements)
            return state
        raise KeyError('pop from an empty priority queue')


def shortest_path_astar(start, end, my_grid):
    if not my_grid.check_collision(start, start) or not my_grid.check_collision(end, end):
        print('The start or end of the path is within a static obstacle radius')
        return False, None
    start_cell = my_grid.get_cell(coords=start)
    end_cell = my_grid.get_cell(coords=end)
    open_queue = PriorityQueue()
    open_cost_so_far = {}
    came_from = {}
    h = my_grid.get_heuristic_distance(start_cell, end_cell)
    open_queue.push(start_cell, h, h)
    came_from[start_cell] = None
    open_cost_so_far[start_cell] = 0
    success = False
    path = []
    while not open_queue.empty():
        current = open_queue.pop()
        if current == end_cell:
            success = True
            break
        neighbors = my_grid.get_neighbors(current)
        current_cost = open_cost_so_far[current]
        for neighbor in neighbors:
            if my_grid.free_path(current, neighbor):
                cost_to_go = current_cost + my_grid.get_travel_cost(current, neighbor)
                if neighbor not in open_cost_so_far or open_cost_so_far[neighbor] > cost_to_go:
                    open_cost_so_far[neighbor] = cost_to_go
                    came_from[neighbor] = current
                    h = my_grid.get_heuristic_distance(neighbor, end_cell)
                    open_queue.push(neighbor, cost_to_go + h, h)
    if success:
        path.append(current.coords)
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current.coords)
        path.reverse()
    return success, path


def smooth_path(path, my_grid):
    new_path = [path[0]]
    for i in range(2, len(path)):
        if not my_grid.check_collision(new_path[-1], path[i]):
            new_path.append(path[i - 1])
    new_path.append(path[-1])
    return new_path
