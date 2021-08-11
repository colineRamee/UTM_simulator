import heapq
import math
import numpy as np
import gurobipy as grb
from gurobipy import GRB
from utm_simulator.my_utils import get_trajectory_circle_intersection, clamp


# For details about the implementation of a priority queue in python see # https://docs.python.org/2/library/heapq.html
class PriorityQueue:
    def __init__(self):
        self.elements = []
        self.priority3 = None

    def empty(self):
        return len(self.elements) == 0

    def push(self, state, priority, priority2, priority3=None):
        # not checking for duplicates. According to redblob games, actually faster than implementing a smarter heapq
        # to break the ties between two cells with same f, favors the one with lowest h
        self.priority3 = priority3
        if self.priority3 is not None:
            heapq.heappush(self.elements, (priority, priority2, priority3, state))
        else:
            heapq.heappush(self.elements, (priority, priority2, state))

    def pop(self):
        # return the lowest priority task
        while self.elements:
            if self.priority3 is not None:
                priority, priority2, priority3, state = heapq.heappop(self.elements)
            else:
                priority, priority2, state = heapq.heappop(self.elements)
            return state
        raise KeyError('pop from an empty priority queue')


class A_star:
    def __init__(self, start_t, goal, map):
        '''Start should be a cell_t, and end a cell'''
        self.start = start_t
        self.goal = goal
        self.map = map
        self.d = math.sqrt(3) * self.map.radius
        self.vel = self.d / self.map.time_step_length

    def heuristic(self, hext):
        # Convert to cube coordinates
        hex = hext.hex
        pos_x = hex.coord[1] - (hex.coord[0] - (hex.coord[0] % 2)) / 2
        pos_y = hex.coord[0]
        pos_z = -pos_x - pos_y
        goal_x = self.goal.coord[1] - (self.goal.coord[0] - (self.goal.coord[0] % 2)) / 2
        goal_y = self.goal.coord[0]
        goal_z = -goal_x - goal_y
        h = (abs(pos_x - goal_x) + abs(pos_y - goal_y) + abs(pos_z - goal_z)) / 2
        h_time_steps = self.d * h / (self.vel * self.map.time_step_length)  # Right now h_time_steps=h because it takes 1 time step to travel one unit
        return h_time_steps

    def search(self):
        open_queue = PriorityQueue()
        open_queue.push(self.start, self.heuristic(self.start), self.heuristic(self.start))
        open_cost_so_far = {}
        came_from = {}
        open_cost_so_far[self.start] = 0
        came_from[self.start] = None
        success = True
        if open_queue.empty():
            print('pathPlanning: A star failed to find a path')
            success = False
        while not open_queue.empty():
            current = open_queue.pop()
            if current.hex == self.goal:
                break
            for neighbor in self.map.getAccessibleStates(current):
                new_cost = open_cost_so_far[current] + self.map.travel_cost(current, neighbor)
                if neighbor not in open_cost_so_far or open_cost_so_far[neighbor] > new_cost:
                    open_cost_so_far[neighbor] = new_cost
                    open_queue.push(neighbor, new_cost + self.heuristic(neighbor), self.heuristic(neighbor))
                    came_from[neighbor] = current
            if open_queue.empty():
                print('pathPlanning: A star failed to find a path')
                success = False
        plan = []
        times = []
        if success:
            plan.append([current.hex.coord[0], current.hex.coord[1]])
            times.append(current.time_step)
            parent = came_from[current]
            stop = False
            while parent is not None and (not stop):
                plan.append([parent.hex.coord[0], parent.hex.coord[1]])
                times.append(parent.time_step)
                if parent.hex.coord[0] == self.start.hex.coord[0] and parent.hex.coord[1] == self.start.hex.coord[1]:
                    stop = True
                parent = came_from[parent]
            times.reverse()
            plan.reverse()
        return success, plan, times


class AStar_8grid:
    def __init__(self, start, goal, start_time, speed, centralized_manager, start_grid=None, end_grid=None):
        self.start = start
        self.goal = goal
        self.centralized_manager = centralized_manager
        self.start_time = start_time
        self.speed = speed
        if start_grid is None:
            self.start_grid = start
        else:
            self.start_grid = start_grid

        if end_grid is None:
            self.end_grid = goal
        else:
            self.end_grid = end_grid
        step = self.centralized_manager.minimum_separation_distance
        self.grid = Grid(self.start_grid, self.end_grid, step, self.start_time, self.speed)
        self.end_cell = self.grid.get_cell(self.goal)
        self.start_cell = self.grid.get_cell(self.start)
        print('A star error at beginning ', np.linalg.norm(self.start_cell.position - self.start))

    def heuristic(self, cell_t):
        remaining_distance = self.grid.estimate_remaining_cost(cell_t.cell, self.end_cell)
        return remaining_distance / self.speed

    def search(self):
        start_cell_t = self.start_cell.get_cell_at_t(self.start_time)
        open_queue = PriorityQueue()
        open_queue.push(start_cell_t, self.heuristic(start_cell_t), self.heuristic(start_cell_t), self.heuristic(start_cell_t))
        open_cost_so_far = {}
        open_energy_so_far = {}
        came_from = {}
        open_cost_so_far[start_cell_t] = 0
        open_energy_so_far[start_cell_t] = 0
        came_from[start_cell_t] = None
        success = True
        if open_queue.empty():
            print('pathPlanning: A star failed to find a path, nothing in the open_queue')
            success = False
        count = 0
        while not open_queue.empty():
            count += 1
            current = open_queue.pop()
            # print('The current node is ' + str(current.cell.position) + ' at ' + str(current.t))
            # print(' f '+str(current.t+self.heuristic(current))+ ', h '+str(self.heuristic(current)))
            if current.cell == self.end_cell:
                break
            for neighbor in self.grid.get_neighbours(current):
                if neighbor.t == current.t:
                    print("this should not happen, a star 8 connected search")
                on_the_ground = (current.cell == self.start_cell) and (neighbor.cell == self.start_cell)
                if on_the_ground or self.centralized_manager.is_path_available(current.t, current.cell.position, neighbor.cell.position, neighbor.t):
                    cost_to_go, energy_to_go = self.grid.travel_cost(current.cell, neighbor.cell)
                    new_energy = open_energy_so_far[current] + energy_to_go
                    new_cost = open_cost_so_far[current] + cost_to_go
                    if neighbor not in open_cost_so_far or open_cost_so_far[neighbor] > new_cost or (open_cost_so_far[neighbor] == new_cost and open_energy_so_far[neighbor] > new_energy):
                        open_cost_so_far[neighbor] = new_cost
                        open_energy_so_far[neighbor] = new_energy
                        open_queue.push(neighbor, new_cost + self.heuristic(neighbor), new_energy + self.heuristic(neighbor), self.heuristic(neighbor))
                        came_from[neighbor] = current
            if open_queue.empty():
                print('pathPlanning: A star failed to find a path')
                success = False
        plan = []
        times = []
        if success:
            print('The a star 8 algo opened ' + str(count) + ' nodes to visit')
            plan.append(current.cell.position)
            times.append(current.t)
            parent = came_from[current]
            stop = False
            while parent is not None and (not stop):
                plan.append(parent.cell.position)
                times.append(parent.t)
                if parent.cell.position[0] == self.start[0] and parent.cell.position[1] == self.start[1]:
                    stop = True
                if came_from[parent] is not None and np.linalg.norm(parent.cell.position - came_from[parent].cell.position) == 0:
                    print('waiting in place')
                parent = came_from[parent]
            times.reverse()
            plan.reverse()
        return success, plan, times


class SIPP:
    def __init__(self, start, goal, start_time, speed, centralized_manager, tolerance):
        self.start = start
        self.goal = goal
        self.centralized_manager = centralized_manager
        self.tolerance = tolerance
        self.i_goal, self.j_goal = self.centralized_manager.get_coordinates(self.goal)
        self.start_time = start_time
        self.speed = speed

    def heuristic(self, state):
        # Need to use diagonal distance!
        d = self.centralized_manager.cell_length
        d_diag = 1.4142135623730951 * d
        i_node, j_node = self.centralized_manager.get_coordinates(state.position)
        dx = abs(i_node - self.i_goal)
        dy = abs(j_node - self.j_goal)
        distance = d * (dx + dy) + (d_diag - 2 * d) * min(dx, dy)
        return distance / self.speed

    def search(self, debug=False):
        open_queue = PriorityQueue()
        open_cost_so_far = {}
        came_from = {}
        # Initialization
        successors = self.centralized_manager.get_successors(self.start, self.start_time, self.speed, on_the_ground=True)
        for successor in successors:
            key = (successor[0][0], successor[0][1], successor[1][0], successor[1][1])
            g = successor[2]
            open_cost_so_far[key] = 0
            start_state = SafeState(successor[0], successor[2])
            came_from[start_state] = None
            h = self.heuristic(start_state)
            open_queue.push(start_state, g + h, h)
        success = True
        count = 0
        on_the_ground = False
        while not open_queue.empty():
            current = open_queue.pop()
            count += 1
            if np.linalg.norm(current.position - self.goal) <= self.tolerance and np.linalg.norm(current.position - self.start) != 0:
                break
            successors = self.centralized_manager.get_successors(current.position, current.time, self.speed, debug=debug)
            for successor in successors:
                key = (successor[0][0], successor[0][1], successor[1][0], successor[1][1])
                g = successor[2]
                if key not in open_cost_so_far or open_cost_so_far[key] > g:
                    open_cost_so_far[key] = g
                    new_state = SafeState(successor[0], successor[2])
                    came_from[new_state] = current
                    h = self.heuristic(new_state)
                    open_queue.push(new_state, g + h, h)
            if open_queue.empty():
                success = False
                print('pathPlanning: SIPP failed to find a path')
                print('start ' + str(self.start))
                print('goal ' + str(self.goal))
                print('came from ' + str(len(came_from)))
                print('Successor start ' + str(self.centralized_manager.get_successors(self.start, self.start_time, self.speed, on_the_ground=True)))
                print('Current position ' + str(current.position) + ', time ' + str(current.time))
                print('Current successors ' + str(self.centralized_manager.get_successors(current.position, current.time, self.speed, debug=True)))
        plan = []
        times = []
        if success:
            direction = self.goal - current.position
            d = np.linalg.norm(direction)
            if d == self.tolerance:
                # Travel a bit inside to make sure it gets deleted
                extra = 0.05 * self.tolerance * direction / d
                plan.append(current.position + extra)
                times.append(current.time + 0.05 * self.tolerance / self.speed)

            position = current.position
            plan.append(position)
            time = current.time
            times.append(time)
            parent = came_from[current]
            stop = False
            while parent is not None and (not stop):
                old_time = parent.time
                old_position = parent.position
                t_intermediate = time - np.linalg.norm(position - old_position) / self.speed
                # Floating point error
                if abs(t_intermediate - old_time) > 1e-5:
                    # it means the agent must wait in place
                    plan.append(old_position)
                    times.append(t_intermediate)
                if parent.position[0] == self.start[0] and parent.position[1] == self.start[1]:
                    stop = True
                    if abs(t_intermediate - old_time) <= 1e-5:
                        plan.append(old_position)
                        times.append(old_time)
                else:
                    plan.append(old_position)
                    times.append(old_time)

                time = old_time
                position = old_position
                parent = came_from[parent]
            times.reverse()
            plan.reverse()
        return success, plan, times


class SafeState:
    def __init__(self, position, time):
        self.position = np.array(position)
        self.time = time

    def __lt__(self, other):
        return self.time < other.time


class Grid:
    def __init__(self, start_grid, end_grid, step, start_time, velocity):
        self.start_time = start_time
        x0 = min(start_grid[0], end_grid[0])
        y0 = min(start_grid[1], end_grid[1])
        x1 = max(start_grid[0], end_grid[0])
        y1 = max(start_grid[1], end_grid[1])
        self.grid_zero = np.array([x0, y0])
        delta_y = y1 - y0
        delta_x = x1 - x0
        self.n_col = int(round(delta_x / step))
        self.step_size_col = delta_x / self.n_col
        self.n_col += 1
        self.n_row = int(round(delta_y / step))
        self.step_size_row = delta_y / self.n_row
        self.n_row += 1
        self.diag_distance = math.sqrt(self.step_size_row ** 2 + self.step_size_col ** 2)
        self.velocity = velocity
        self.cells = np.empty((self.n_row + 1, self.n_col + 1), dtype=object)
        self.hover_time = ((self.step_size_col + self.step_size_row) / 2) / self.velocity

    def get_cell(self, position):
        vector = position - self.grid_zero
        index_i = int(round(vector[1] / self.step_size_row))
        index_j = int(round(vector[0] / self.step_size_col))
        return self.get_cell_at(index_i, index_j)

    def get_position(self, i, j):
        return self.grid_zero + np.array([self.step_size_col * j, self.step_size_row * i])

    def get_cell_at(self, i, j):
        if self.cells[i, j] is None:
            self.cells[i, j] = Cell(self.get_position(i, j), i, j)
        return self.cells[i, j]

    def travel_cost(self, cell_a, cell_b):
        d = np.linalg.norm(cell_a.position - cell_b.position)
        if d != 0:
            time = d / self.velocity
        else:
            time = self.hover_time
        return time, d

    def estimate_remaining_cost(self, cell, goal):
        """Diagonal distance for 8-connected grid"""
        dx = abs(cell.i - goal.i)
        dy = abs(cell.j - goal.j)
        # Diagonal distance
        return self.step_size_col * dx + self.step_size_row * dy + (self.diag_distance - self.step_size_row - self.step_size_col) * min(dx, dy)

    def get_neighbours(self, my_cell_t):
        i = my_cell_t.cell.i
        j = my_cell_t.cell.j
        i_array = [i]
        j_array = [j]
        neighbours = []
        if i > 0:
            i_array.append(i - 1)
        if i < self.n_row:
            i_array.append(i + 1)
        if j > 0:
            j_array.append(j - 1)
        if j < self.n_col:
            j_array.append(j + 1)
        for i_index in i_array:
            for j_index in j_array:
                spatial_neighbour = self.get_cell_at(i_index, j_index)
                t, d = self.travel_cost(my_cell_t.cell, spatial_neighbour)
                neighbours.append(spatial_neighbour.get_cell_at_t(my_cell_t.t + t))
        return neighbours


class Cell:
    def __init__(self, position, i, j):
        self.position = position
        self.i = i
        self.j = j
        self.time_dic = {}

    def get_cell_at_t(self, t):
        if t in self.time_dic:
            return self.time_dic[t]
        else:
            return Cell_t(self, t)


class Cell_t:
    def __init__(self, cell, t):
        self.cell = cell
        self.t = t
        self.cell.time_dic[t] = self

    def __lt__(self, other):
        return self.cell.position[0] < other.cell.position[0]


# Local Method
class Local_VO:
    def __init__(self, start, goal, start_time, speed, centralized_manager, tolerance):
        self.start = start
        self.goal = goal
        self.centralized_manager = centralized_manager
        self.tolerance = tolerance
        self.start_time = start_time
        self.speed = speed
        self.sensing_radius = 5000
        self.minimum_separation_distance = centralized_manager.minimum_separation_distance
        self.dt = 10
        self.k_nearest_neighbors = 10
        self.k_closest_obstacles = 5
        self.maneuvering_distance_obstacle = 1000
        self.safe_distance = self.dt * self.speed
        self.lookahead_distance = 500

    def search(self, debug=False):
        success = False
        if debug:
            print('Starting debugging')
        # Plan a path that avoids static obstacles (if they exist)
        if self.centralized_manager.static_obstacles != []:
            path = self.centralized_manager.get_path_smoothed(self.start, self.goal)
        else:
            path = None
        while not success:
            # check if you can take-off (i.e. there are no vehicles forecast right above you). If cannot take-off, wait for dt and check again
            pos = np.copy(self.start)
            intruders_pos, intruders_vel = self.centralized_manager.get_forecast_intruders(pos, self.minimum_separation_distance, self.start_time)
            while len(intruders_pos) != 0:
                if debug:
                    print('adding ground delay')
                self.start_time += self.dt
                intruders_pos, intruders_vel = self.centralized_manager.get_forecast_intruders(pos, self.minimum_separation_distance, self.start_time)
            time = self.start_time
            planned_times = [time]
            planned_positions = [np.copy(pos)]
            direction_to_goal = self.goal - pos
            distance_to_goal = np.linalg.norm(direction_to_goal)
            no_solution = False
            while distance_to_goal > self.tolerance and not no_solution:
                positions, velocities = self.centralized_manager.get_forecast_intruders(pos, self.sensing_radius, time)
                # Let's ignore all static obstacles that are beyond the goal
                max_obstacle_sensing_radius = min(self.maneuvering_distance_obstacle, distance_to_goal)
                obstacles, safe_index = self.centralized_manager.get_static_obstacles(pos, max_obstacle_sensing_radius, self.safe_distance)
                if path is None:
                    desired_vel = min(self.speed, distance_to_goal / self.dt) * direction_to_goal / distance_to_goal
                else:
                    lookahead_point = self.get_lookahead_on_path(path, pos)
                    distance_to_lookahead = np.linalg.norm(lookahead_point - pos)
                    direction_to_lookahead = lookahead_point - pos
                    desired_vel = min(self.speed, distance_to_lookahead / self.dt) * direction_to_lookahead / distance_to_lookahead
                if len(positions) + len(obstacles) != 0:
                    # only consider the k closest intruders (get forecast intruder returns a sorted list, and same for static obs)
                    k_max_intruders = min(self.k_nearest_neighbors, len(positions))
                    if safe_index is not None:
                        k_max_obstacles = min(max(self.k_closest_obstacles, safe_index + 1), len(obstacles))
                    else:
                        k_max_obstacles = min(self.k_closest_obstacles, len(obstacles))
                    model = self.setupMIQCP(positions[0:k_max_intruders], velocities[0:k_max_intruders], desired_vel, pos, obstacles=obstacles[0:k_max_obstacles])
                    if model is None:
                        no_solution = True
                        break
                    model.optimize()
                    if model.status != GRB.Status.OPTIMAL:
                        if debug:
                            print('Error gurobi failed to find a solution')
                            print(model.status)
                        no_solution = True
                        break
                    vars = model.getVars()
                    opt_vel = np.array([vars[0].x, vars[1].x])
                    path_available = self.centralized_manager.is_path_available(time, pos, pos + opt_vel * self.dt, time + self.dt)  # Can be False if agents change heading within self.dt
                    if not path_available:
                        no_solution = True
                        break
                    if debug:
                        print('Optimized velocity')
                        print(opt_vel)
                        print(np.linalg.norm(opt_vel))
                        print('Trajectory ' + str(planned_positions))
                        print('A* path ' + str(path))
                    if np.linalg.norm(opt_vel) < 0.01 and k_max_obstacles != 0:
                        # Might be stuck in a local minima due to static obstacles, check if that's the case and if it is terminate the planning
                        difference, direction = self.find_closest_available_direction_static(pos, desired_vel, obstacles[0:k_max_obstacles])
                        if difference is None:
                            print('No available direction in static, the planning is stuck')
                            print('Number of obstacles being considered ' + str(k_max_obstacles))
                            return False, [], []
                        elif difference <= 80 * math.pi / 180:
                            # The desired direction or a direction close to it is available in static, an active agent is blocking the path, waiting a bit will take care of the issue
                            if debug:
                                print('Optimized velocity is small but the path is not blocked by a static obstacle')
                            pass
                        else:
                            # There is a static obstacle in the way, this should only happen if the agent has significantly deviated from its planned path
                            # Restart planning with a shortest lookahead distance
                            if debug:
                                print('Optimized velocity is small and the path is blocked by a static obstacle')
                                print('Path ' + str(path))
                                print('positions ' + str(planned_positions))
                                print('The lookahead position is ' + str(lookahead_point))
                                print('The total distance to the lookahead point is ' + str(np.linalg.norm(path[1] - planned_positions[-1]) + np.linalg.norm(path[1] - lookahead_point)))
                            no_solution = True
                            break
                            # Heuristic 2 (with global planning)
                            # Heuristic 1. switch things up to find a solution
                            # intensity = np.linalg.norm(desired_vel)
                            # desired_vel = intensity * np.array([math.cos(direction), math.sin(direction)])
                            # model = self.setupMIQCP(positions[0:k_max_intruders], velocities[0:k_max_intruders], desired_vel, pos, obstacles=obstacles[0:k_max_obstacles])
                            # model.optimize()
                            # if model.status != GRB.Status.OPTIMAL:
                            #     if debug:
                            #         print('Error gurobi failed to find a solution after switching directions')
                            #         print(model.status)
                            #     no_solution = True
                            #     break
                            # vars = model.getVars()
                            # opt_vel = np.array([vars[0].x, vars[1].x])
                            # if debug:
                            #     print('Planned positions')
                            #     print(planned_positions)
                            #     print('start '+str(self.start))
                            #     print('goal ' + str(self.goal))
                else:
                    # If there are no intruders nearby, no need to optimize
                    if debug:
                        print('no intruders')
                    opt_vel = desired_vel
                planned_positions.append(pos + opt_vel * self.dt)
                planned_times.append(time + self.dt)
                pos = pos + opt_vel * self.dt
                time = time + self.dt
                direction_to_goal = self.goal - pos
                distance_to_goal = np.linalg.norm(direction_to_goal)
            if no_solution:
                # Try again at a later time (put it in a loop or call search iteratively?)
                if debug:
                    print('Try again with a later start')
                success = False
                self.start_time += self.dt
            else:
                success = True
        if not success:
            print('Agent failed to plan')
        return success, planned_positions, planned_times

    def get_lookahead_on_path(self, path, ownship_position):
        previous_point = path[0]
        min_distance = None
        min_position = None
        index = None
        n = len(path)
        for i in range(1, n):
            # Find closest point to position on path[i-1]-path[i]
            current_point = path[i]
            segment_direction = current_point - previous_point
            # find distance
            t = - np.dot(segment_direction, previous_point - ownship_position) / np.dot(segment_direction, segment_direction)
            t = clamp(0, 1, t)
            position = previous_point + t * segment_direction
            distance = np.linalg.norm(position - ownship_position)
            # Found distance
            if min_distance is None or min_distance > distance:
                min_distance = distance
                index = i
                min_position = position
            previous_point = current_point
        # Now find a point that is a lookahead distance on the path of the min_position
        keep_iterating = True
        distance_to_cover = self.lookahead_distance
        while keep_iterating:
            if index >= n:
                # Reached the end of the path
                lookahead_point = path[-1]
                keep_iterating = False
            else:
                segment_length = np.linalg.norm(path[index] - min_position)
                if distance_to_cover > segment_length:
                    # Move to the next segment
                    distance_to_cover -= segment_length
                    min_position = path[index]
                    index += 1
                else:
                    # The lookahead point is on this segment
                    lookahead_point = min_position + distance_to_cover * (path[index] - min_position) / segment_length
                    keep_iterating = False
        return lookahead_point

    def find_closest_available_direction_static(self, ownship_pos, opt_vel, obstacles, debug=False):
        available_directions = [[-math.pi, math.pi]]
        current_desired_direction = math.atan2(opt_vel[1], opt_vel[0])
        for obstacle in obstacles:
            rel_pos = obstacle.position - ownship_pos
            d = np.linalg.norm(rel_pos)
            alpha = math.asin(obstacle.radius / d)  # VO cone half-angle (>=0)
            theta = math.atan2(rel_pos[1], rel_pos[0])
            lower_bound = ((theta - alpha + math.pi) % (2 * math.pi)) - math.pi
            upper_bound = ((theta + alpha + math.pi) % (2 * math.pi)) - math.pi
            if lower_bound < upper_bound:
                occupied_intervals = [[lower_bound, upper_bound]]
            else:
                occupied_intervals = [[lower_bound, math.pi], [-math.pi, upper_bound]]
            if debug:
                print('occupied interval by the obstacle ' + str(occupied_intervals))
            for occupied_interval in occupied_intervals:
                keep_iterating = True
                i = 0
                n_intervals = len(available_directions)
                if debug:
                    print('starting analysis for one occupied interval')
                while keep_iterating:
                    free_interval = available_directions[i]
                    if free_interval[0] < occupied_interval[1] and occupied_interval[0] < free_interval[1]:
                        # The intervals overlap
                        if free_interval[0] < occupied_interval[0] and occupied_interval[1] < free_interval[1]:
                            # occupied interval is entirely inside one free interval
                            available_directions.pop(i)
                            available_directions.insert(i, [free_interval[0], occupied_interval[0]])
                            available_directions.insert(i + 1, [occupied_interval[1], free_interval[1]])
                            keep_iterating = False
                        elif occupied_interval[0] <= free_interval[0] and free_interval[1] <= occupied_interval[1]:
                            # occupied_interval covers free interval entirely
                            available_directions.pop(i)
                            n_intervals -= 1
                        elif occupied_interval[0] <= free_interval[0]:
                            free_interval[0] = occupied_interval[1]
                            i += 1
                        elif free_interval[1] <= occupied_interval[1]:
                            free_interval[1] = occupied_interval[0]
                            i += 1
                        else:
                            print('This should not happen finding available directions')
                            print(i)
                            print('free interval ' + str(free_interval))
                            print(occupied_interval)
                    else:
                        i += 1
                    if i >= n_intervals or free_interval[1] >= occupied_interval[1]:
                        keep_iterating = False
                if debug:
                    print('New free intervals ' + str(available_directions))
        # Now figure out what the closest free interval is to the current desired_direction
        best_diff = None
        best_direction = None
        for interval in available_directions:
            if interval[0] <= current_desired_direction <= interval[1]:
                best_direction = current_desired_direction
                best_diff = 0
                break
            else:
                diff_a = abs(((current_desired_direction - interval[0] + math.pi) % (2 * math.pi)) - math.pi)
                diff_b = abs(((current_desired_direction - interval[1] + math.pi) % (2 * math.pi)) - math.pi)
                if diff_a < diff_b:
                    direction = interval[0]
                    diff = diff_a
                else:
                    direction = interval[1]
                    diff = diff_b
                if best_diff is None or diff < best_diff:
                    best_direction = direction
                    best_diff = diff
        return best_diff, best_direction

    def setupMIQCP(self, intruders_pos, intruders_vel, desired_vel, ownship_pos, obstacles=[]):
        """ """
        model = grb.Model('VO')
        model.addVar(lb=-self.speed, ub=self.speed, name='x')
        model.addVar(lb=-self.speed, ub=self.speed, name='y')
        n_obstacles = len(obstacles)
        n_intruders = len(intruders_pos)
        model.addVars(2 * (n_obstacles + n_intruders), vtype=GRB.BINARY)
        model.update()
        X = model.getVars()
        n_intruder = 0
        for i in range(0, n_obstacles + n_intruders):
            if i < n_obstacles:
                constraints_or = self.get_static_VO(obstacles[i], ownship_pos)
            else:
                constraints_or = self.get_VO(intruders_pos[i - n_obstacles], intruders_vel[i - n_obstacles], ownship_pos)
            if constraints_or[0] is None:
                return None
            n_constraint = 0
            for constraint in constraints_or:
                c = constraint(0, 0)
                a = constraint(1, 0) - c
                b = constraint(0, 1) - c
                # K must be arbitrarily large so that when the binary constraint is 1 the constraint is always respected
                # If K is chosen too large it creates issues for the solver. K is chosen just large enough.
                K = abs(a * self.speed) + abs(b * self.speed) + c
                model.addConstr(a * X[0] + b * X[1] - K * X[2 + 2 * n_intruder + n_constraint] <= -c)
                n_constraint += 1
            model.addConstr(X[2 + 2 * n_intruder] + X[2 + 2 * n_intruder + 1] <= 1)
            n_intruder += 1
        model.addConstr(X[0] * X[0] + X[1] * X[1] <= self.speed ** 2)
        model.setObjective((X[0] - desired_vel[0]) * (X[0] - desired_vel[0]) + (X[1] - desired_vel[1]) * (X[1] - desired_vel[1]), GRB.MINIMIZE)
        model.setParam("OutputFlag", 0)
        model.setParam("Threads", 1)
        model.setParam("FeasibilityTol", 1e-9)
        model.update()
        return model

    def get_VO(self, intruder_pos, intruder_vel, ownship_pos):
        rel_pos = intruder_pos - ownship_pos
        d = np.linalg.norm(rel_pos)
        if self.minimum_separation_distance > d:
            # There is a loss of separation
            return None, None
        alpha = math.asin(self.minimum_separation_distance / d)  # VO cone half-angle (>=0)
        theta = math.atan2(rel_pos[1], rel_pos[0])
        vector1 = [math.cos(theta + alpha), math.sin(theta + alpha)]
        vector2 = [math.cos(theta - alpha), math.sin(theta - alpha)]
        # must be greater
        normal_1 = np.array([vector1[1], -vector1[0]])  # Rotated +90 degrees
        constraint1 = lambda x, y: np.dot((np.array([x, y]) - intruder_vel) + 0.1 * normal_1, normal_1)
        # must be smaller
        normal_2 = np.array([-vector2[1], vector2[0]])  # Rotated -90 degrees
        constraint2 = lambda x, y: np.dot((np.array([x, y]) - intruder_vel) + 0.1 * normal_2, normal_2)
        return constraint1, constraint2

    def get_static_VO(self, obstacle, ownship_pos):
        rel_pos = obstacle.position - ownship_pos
        d = np.linalg.norm(rel_pos)
        if obstacle.radius > d:
            # There is a loss of separation
            return None, None
        alpha = math.asin(obstacle.radius / d)  # VO cone half-angle (>=0)
        theta = math.atan2(rel_pos[1], rel_pos[0])
        vector1 = [math.cos(theta + alpha), math.sin(theta + alpha)]
        vector2 = [math.cos(theta - alpha), math.sin(theta - alpha)]
        # must be greater
        normal_1 = np.array([vector1[1], -vector1[0]])  # Rotated +90 degrees
        constraint1 = lambda x, y: np.dot(np.array([x, y]) + 0.001 * normal_1, normal_1)
        # must be smaller
        normal_2 = np.array([-vector2[1], vector2[0]])  # Rotated -90 degrees
        constraint2 = lambda x, y: np.dot(np.array([x, y]) + 0.001 * normal_2, normal_2)
        return constraint1, constraint2
