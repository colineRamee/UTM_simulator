import numpy as np
import math
from utm_simulator.my_utils import clamp
import utm_simulator.staticPathPlanning
from scipy.sparse import lil_matrix


class CentralizedManager:
    def __init__(self, min_separation, size_area=None, static_obstacles=None):
        self.flight_plans = {}
        self.agents = {}
        self.minimum_separation_distance = min_separation
        self.size_area = size_area
        self.grid = None
        if static_obstacles is None:
            self.static_obstacles = []
        else:
            self.static_obstacles = static_obstacles
            if size_area is None:
                print('With static obstacles, an area size must be specified when creating the centralized manager')

    def add_static_obstacles(self, static_obstacles_list):
        self.static_obstacles = static_obstacles_list

    def get_shortest_path_static(self, start, end):
        if self.static_obstacles == []:
            return [start, end]
        if self.grid is None:
            self.grid = utm_simulator.staticPathPlanning.Grid(self.minimum_separation_distance, self.size_area, self.static_obstacles)
        success, path = utm_simulator.staticPathPlanning.shortest_path_astar(start, end, self.grid)
        if not success:
            print('Error centralized manager get_shortest_path_static: no path found')
        return path

    def get_path_smoothed(self, start, end):
        path = self.get_shortest_path_static(start, end)
        return utm_simulator.staticPathPlanning.smooth_path(path, self.grid)

    def is_path_available(self, ownship_start_time, start_point, end_point, end_time):
        # How should collision avoidance be implemented ?
        # Naive approach
        # Iterate over all flight plans, find all flight segments that overlap in time with the input time
        # Compute for each of those segments the closest time of approach
        # Need to intersect time between time_start and time_end and get the resulting trajectory
        ownship_velocity = (end_point - start_point) / (end_time - ownship_start_time)
        ownship_velocity_squared = np.dot(ownship_velocity, ownship_velocity)
        for obstacle in self.static_obstacles:
            if ownship_velocity_squared != 0:
                t = -np.dot(ownship_velocity, start_point - obstacle.position) / ownship_velocity_squared
                t = clamp(0, end_time - ownship_start_time, t)
            else:
                t = 0
            min_distance = np.linalg.norm(start_point + t * ownship_velocity - obstacle.position)
            if min_distance < obstacle.radius:
                return False
        for flight_plan in self.flight_plans.values():
            trajectory, times = flight_plan.get_planned_trajectory_between(ownship_start_time, end_time)
            if trajectory is not None:
                n = len(trajectory)
                intruder_segment_start_pos = np.copy(trajectory[0])
                intruder_segment_start_time = times[0]
                analysis_start_time = max(ownship_start_time, intruder_segment_start_time)
                ownship_pos = start_point + (analysis_start_time - ownship_start_time) * ownship_velocity
                for i in range(1, n):
                    intruder_segment_end_pos = trajectory[i]
                    intruder_segment_end_time = times[i]
                    if intruder_segment_end_time == intruder_segment_start_time:
                        print('trajectory ' + str(trajectory))
                        print('times ' + str(times))
                        print('ownship start time ' + str(ownship_start_time))
                        print('end time ' + str(end_time))
                    intruder_velocity = (intruder_segment_end_pos - intruder_segment_start_pos) / (intruder_segment_end_time - intruder_segment_start_time)
                    intruder_pos = intruder_segment_start_pos + (analysis_start_time - intruder_segment_start_time) * intruder_velocity
                    # Let's compute the closest time of approach
                    delta_v = intruder_velocity - ownship_velocity
                    delta_v_squared = np.dot(delta_v, delta_v)
                    delta_p = intruder_pos - ownship_pos
                    if delta_v_squared != 0:
                        t_cpa = analysis_start_time - np.dot(delta_v, delta_p) / delta_v_squared
                        # Let's find the time of closest approach on the segment
                        clamped_t_cpa = clamp(analysis_start_time, intruder_segment_end_time, t_cpa) - analysis_start_time  # between 0 and the end of the segment (t_elapsed)
                    else:
                        clamped_t_cpa = 0
                    closest = np.linalg.norm(intruder_pos + intruder_velocity * clamped_t_cpa - ownship_pos - ownship_velocity * clamped_t_cpa)
                    if closest < self.minimum_separation_distance:
                        return False
                    intruder_segment_start_time = intruder_segment_end_time
                    intruder_segment_start_pos = intruder_segment_end_pos
                    analysis_start_time = max(ownship_start_time, intruder_segment_start_time)
                    ownship_pos = start_point + (analysis_start_time - ownship_start_time) * ownship_velocity
        return True

    def add_flight_plan(self, flight_plan, my_id, agent, my_time, priority=False):
        perturbed_agents_in_air = []
        perturbed_agents_ground = []
        if priority:
            # The new flight plan has priority over all flight plans
            # For now priority agents have constant velocity this can be used to determine closest point of approach without having to iterate on the priority agent flight plan
            # Iterate over all flight plans
            for id in self.flight_plans:
                # retrieve the trajectory between the two times
                if self.agents[id].agent_logic != 'dumb':
                    trajectory, times = self.flight_plans[id].get_planned_trajectory_between(flight_plan.times[0], flight_plan.times[-1])
                    if times is not None:
                        for i in range(0, len(times) - 1):
                            priority_position, priority_velocity = flight_plan.get_planned_position_at(times[i], return_velocity=True)
                            vel = (trajectory[i + 1] - trajectory[i]) / (times[i + 1] - times[i])
                            delta_p = trajectory[i] - priority_position
                            delta_v = vel - priority_velocity
                            delta_v_squared = np.dot(delta_v, delta_v)
                            if delta_v_squared != 0:
                                t_cpa = - np.dot(delta_p, delta_v) / delta_v_squared
                            else:
                                t_cpa = 0
                            clamped_t_cpa = clamp(0, times[i + 1] - times[i], t_cpa)
                            closest = np.linalg.norm(delta_p + clamped_t_cpa * delta_v)
                            if closest < self.minimum_separation_distance:
                                # Keep a list of agents that will have to replan or yield priority
                                if self.flight_plans[id].times[0] > my_time:
                                    perturbed_agents_ground.append(id)
                                else:
                                    perturbed_agents_in_air.append(id)
                                break
        for id in perturbed_agents_in_air + perturbed_agents_ground:
            del self.flight_plans[id]
        self.flight_plans[my_id] = flight_plan
        self.agents[my_id] = agent
        # Make perturbed agents replan or switch behavior
        for id in perturbed_agents_in_air + perturbed_agents_ground:
            new_flight_plan = self.agents[id].yield_priority(flight_plan, my_time)
            if new_flight_plan is not None:
                self.flight_plans[id] = new_flight_plan

    def terminate_flight_plan(self, id):
        if id in self.flight_plans:
            del self.flight_plans[id]
        if id in self.agents:
            del self.agents[id]
        else:
            print('Centralized Manager Error: Weird! the id should be in the agents list')

    def validate_flight_plan(self, flight_plan):
        old_pos = flight_plan.positions[0]
        old_t = flight_plan.times[0]
        for i in range(1, len(flight_plan.positions)):
            pos = flight_plan.positions[i]
            t = flight_plan.times[i]
            if not self.is_path_available(old_t, old_pos, pos, t):
                return False
            old_pos = pos
            old_t = t
        return True


class VOCentralizedManager:
    """Centralized manager for local path planning"""

    def __init__(self, min_separation, size_area=None, static_obstacles=None):
        self.flight_plans = {}
        self.agents = {}
        self.minimum_separation_distance = min_separation
        self.grid = None
        self.size_area = size_area
        if static_obstacles is None:
            self.static_obstacles = []
        else:
            self.static_obstacles = static_obstacles

    def add_static_obstacles(self, static_obstacles_list):
        self.static_obstacles = static_obstacles_list

    def get_shortest_path_static(self, start, end):
        if self.static_obstacles == []:
            return [start, end]
        if self.grid is None:
            self.grid = utm_simulator.staticPathPlanning.Grid(self.minimum_separation_distance, self.size_area, self.static_obstacles)
        success, path = utm_simulator.staticPathPlanning.shortest_path_astar(start, end, self.grid)
        if not success:
            print('Error centralized manager get_shortest_path_static: no path found')
        return path

    def get_path_smoothed(self, start, end):
        path = self.get_shortest_path_static(start, end)
        return utm_simulator.staticPathPlanning.smooth_path(path, self.grid)

    def get_static_obstacles(self, ownship_pos, radius, safety_distance):
        # Might be a way to speed up the lookup by creating a grid or tree structure that lets you know which obstacle partially cover it
        # For now quick non-optimized lookup of all obstacles (100 ish in worst case scenario)
        visible_static_obs = []
        distance_to_obs_boundary = []
        for obstacle in self.static_obstacles:
            dist = np.linalg.norm(obstacle.position - ownship_pos)
            if dist < obstacle.radius + radius:
                d = dist - obstacle.radius
                i = np.searchsorted(distance_to_obs_boundary, d)
                distance_to_obs_boundary.insert(i, d)
                visible_static_obs.insert(i, obstacle)
        # safe_index is the index of the last sorted obstacle that is within the safety distance of the ownship
        index = np.searchsorted(distance_to_obs_boundary, safety_distance, side='right')
        if index == 0:
            safe_index = None
        else:
            safe_index = index - 1
        return visible_static_obs, safe_index

    def is_path_available(self, ownship_start_time, start_point, end_point, end_time):
        ownship_velocity = (end_point - start_point) / (end_time - ownship_start_time)
        ownship_velocity_squared = np.dot(ownship_velocity, ownship_velocity)
        for obstacle in self.static_obstacles:
            if ownship_velocity_squared != 0:
                t = -np.dot(ownship_velocity, start_point - obstacle.position) / ownship_velocity_squared
                t = clamp(0, end_time - ownship_start_time, t)
            else:
                t = 0
            min_distance = np.linalg.norm(start_point + t * ownship_velocity - obstacle.position)
            if min_distance < obstacle.radius:
                return False
        for flight_plan in self.flight_plans.values():
            trajectory, times = flight_plan.get_planned_trajectory_between(ownship_start_time, end_time)
            if trajectory is not None:
                n = len(trajectory)
                intruder_segment_start_pos = np.copy(trajectory[0])
                intruder_segment_start_time = times[0]
                analysis_start_time = max(ownship_start_time, intruder_segment_start_time)
                ownship_pos = start_point + (analysis_start_time - ownship_start_time) * ownship_velocity
                for i in range(1, n):
                    intruder_segment_end_pos = trajectory[i]
                    intruder_segment_end_time = times[i]
                    if intruder_segment_end_time == intruder_segment_start_time:
                        print('trajectory ' + str(trajectory))
                        print('times ' + str(times))
                        print('ownship start time ' + str(ownship_start_time))
                        print('end time ' + str(end_time))
                    intruder_velocity = (intruder_segment_end_pos - intruder_segment_start_pos) / (intruder_segment_end_time - intruder_segment_start_time)
                    intruder_pos = intruder_segment_start_pos + (analysis_start_time - intruder_segment_start_time) * intruder_velocity
                    # Let's compute the closest time of approach
                    delta_v = intruder_velocity - ownship_velocity
                    delta_v_squared = np.dot(delta_v, delta_v)
                    delta_p = intruder_pos - ownship_pos
                    if delta_v_squared != 0:
                        t_cpa = analysis_start_time - np.dot(delta_v, delta_p) / delta_v_squared
                        # Let's find the time of closest approach on the segment
                        clamped_t_cpa = clamp(analysis_start_time, intruder_segment_end_time, t_cpa) - analysis_start_time  # between 0 and the end of the segment (t_elapsed)
                    else:
                        clamped_t_cpa = 0
                    closest = np.linalg.norm(intruder_pos + intruder_velocity * clamped_t_cpa - ownship_pos - ownship_velocity * clamped_t_cpa)
                    if closest < self.minimum_separation_distance:
                        return False
                    intruder_segment_start_time = intruder_segment_end_time
                    intruder_segment_start_pos = intruder_segment_end_pos
                    analysis_start_time = max(ownship_start_time, intruder_segment_start_time)
                    ownship_pos = start_point + (analysis_start_time - ownship_start_time) * ownship_velocity
        return True

    def add_flight_plan(self, flight_plan, my_id, agent, my_time, priority=False):
        perturbed_agents_in_air = []
        perturbed_agents_ground = []
        if priority:
            # The new flight plan has priority over all flight plans
            # For now priority agents have constant velocity this can be used to determine closest point of approach without having to iterate on the priority agent flight plan
            # Iterate over all flight plans
            for id in self.flight_plans:
                # retrieve the trajectory between the two times
                if self.agents[id].agent_logic != 'dumb':
                    trajectory, times = self.flight_plans[id].get_planned_trajectory_between(flight_plan.times[0], flight_plan.times[-1])
                    if times is not None:
                        for i in range(0, len(times) - 1):
                            priority_position, priority_velocity = flight_plan.get_planned_position_at(times[i], return_velocity=True)
                            vel = (trajectory[i + 1] - trajectory[i]) / (times[i + 1] - times[i])
                            delta_p = trajectory[i] - priority_position
                            delta_v = vel - priority_velocity
                            delta_v_squared = np.dot(delta_v, delta_v)
                            if delta_v_squared != 0:
                                t_cpa = - np.dot(delta_p, delta_v) / delta_v_squared
                            else:
                                t_cpa = 0
                            clamped_t_cpa = clamp(0, times[i + 1] - times[i], t_cpa)
                            closest = np.linalg.norm(delta_p + clamped_t_cpa * delta_v)
                            if closest < self.minimum_separation_distance:
                                # Notify agent that it will have to replan or switch to reactive
                                # Keep a list of agents that will have to replan or yield priority
                                if self.flight_plans[id].times[0] > my_time:
                                    perturbed_agents_ground.append(id)
                                else:
                                    perturbed_agents_in_air.append(id)
                                break
        for id in perturbed_agents_in_air + perturbed_agents_ground:
            del self.flight_plans[id]
        # print('centralize_manager received new flight plan')
        # print(np.array(flight_plan.positions))
        self.flight_plans[my_id] = flight_plan
        self.agents[my_id] = agent
        # Make perturbed agents replan or switch behavior
        for id in perturbed_agents_in_air + perturbed_agents_ground:
            new_flight_plan = self.agents[id].yield_priority(flight_plan, my_time)
            if new_flight_plan is not None:
                self.flight_plans[id] = new_flight_plan

    def terminate_flight_plan(self, id):
        if id in self.flight_plans:
            del self.flight_plans[id]
        if id in self.agents:
            del self.agents[id]
        else:
            print('Error in centralized manager, the id should be in agents list')

    def get_forecast_intruders(self, pos, radius, time, debug=False):
        # TODO return static obstacles too
        distances = []
        positions = []
        velocities = []
        for flight_plan in self.flight_plans.values():
            pos_i, vel_i = flight_plan.get_planned_position_at(time, return_velocity=True, ignore_timed_out=True, debug=debug)
            if pos_i is not None:
                d = np.linalg.norm(pos_i - pos)
                if d < radius:
                    i = np.searchsorted(distances, d)
                    distances.insert(i, d)
                    positions.insert(i, pos_i)
                    velocities.insert(i, vel_i)
        return positions, velocities

    def validate_flight_plan(self, flight_plan):
        old_pos = flight_plan.positions[0]
        old_t = flight_plan.times[0]
        for i in range(1, len(flight_plan.positions)):
            pos = flight_plan.positions[i]
            t = flight_plan.times[i]
            if not self.is_path_available(old_t, old_pos, pos, t):
                return False
            old_pos = pos
            old_t = t
        return True


class SIPPCentralizedManager:
    """Centralized manager implementation for safe-interval path planning"""

    def __init__(self, min_separation, size_area, static_obstacles=None):
        self.flight_plans = {}
        self.minimum_separation_distance = min_separation
        self.size_area = size_area
        self.n_grid = math.ceil(self.size_area / self.minimum_separation_distance)
        self.cell_length = self.size_area / self.n_grid
        self.n_grid += 1
        self.occupancy_grid = None
        self.connectivity_matrix = None
        self.agents = {}
        if static_obstacles is None:
            self.static_obstacles = []
        else:
            self.static_obstacles = static_obstacles

    def initialize_occupancy_grid(self):
        self.occupancy_grid = np.empty((self.n_grid, self.n_grid), dtype='object')
        if self.static_obstacles != []:
            self.connectivity_matrix = lil_matrix((self.n_grid * self.n_grid, self.n_grid * self.n_grid), dtype=bool)
        for i in range(0, self.n_grid):
            for j in range(0, self.n_grid):
                # Not optimized but easy to implement
                cell_center = self.get_position(i, j)
                occupied = False
                obstacles_to_check_for_connectivity = []
                for obstacle in self.static_obstacles:
                    distance = np.linalg.norm(cell_center - obstacle.position)
                    if distance < obstacle.radius:
                        self.occupancy_grid[i, j] = [[]]  # No free interval
                        occupied = True
                        break
                    elif distance < obstacle.radius + self.cell_length * 1.4142135623730951:
                        # Check if one can travel from this cell to a neighboring cell
                        obstacles_to_check_for_connectivity.append(obstacle)
                if not occupied:
                    self.occupancy_grid[i, j] = [[0, float('inf')]]
                    if self.static_obstacles != []:
                        neighbors = []
                        # Test static connectivity to half the neighbors (as we go through the whole grid all connections will be checked)
                        if i < self.n_grid - 1:
                            neighbors.append([i + 1, j])
                        if j < self.n_grid - 1:
                            neighbors.append([i, j + 1])
                            if i < self.n_grid - 1:
                                neighbors.append([i + 1, j + 1])
                            if i > 0:
                                neighbors.append([i - 1, j + 1])
                        for neighbor in neighbors:
                            free_path = True
                            v = self.get_position(neighbor[0], neighbor[1]) - cell_center
                            v_squared = np.dot(v, v)
                            for obstacle in obstacles_to_check_for_connectivity:
                                # find the closest point on the segment cell_center to neighbor to the center of the obstacle
                                t = - np.dot(v, cell_center - obstacle.position) / v_squared
                                t = clamp(0, 1, t)
                                min_distance = np.linalg.norm(cell_center + t * v - obstacle.position)
                                if min_distance < obstacle.radius:
                                    free_path = False
                                    break
                            if free_path:
                                self.connectivity_matrix[i * self.n_grid + j, neighbor[0] * self.n_grid + neighbor[1]] = True
                                self.connectivity_matrix[neighbor[0] * self.n_grid + neighbor[1], i * self.n_grid + j] = True

    def reset_occupancy_grid(self):
        for i in range(0, self.n_grid):
            for j in range(0, self.n_grid):
                # If the available interval is empty, that means there is a static obstacle
                if self.occupancy_grid[i, j] != [[]]:
                    self.occupancy_grid[i, j] = [[0, float('inf')]]
        for agent_id in self.flight_plans:
            self.update_availability_grid(self.flight_plans[agent_id])

    def add_static_obstacles(self, static_obstacles_list):
        self.static_obstacles = static_obstacles_list
        self.initialize_occupancy_grid()

    def get_neighbours(self, position):
        neighbours = []
        i_array = []
        j_array = []
        i, j = self.get_coordinates(position)
        i_array.append(i)
        j_array.append(j)
        if i > 0:
            i_array.append(i - 1)
        if i < self.n_grid - 1:
            i_array.append(i + 1)
        if j > 0:
            j_array.append(j - 1)
        if j < self.n_grid - 1:
            j_array.append(j + 1)
        for i_index in i_array:
            for j_index in j_array:
                if i_index != i or j_index != j:
                    # Check that the path to the neighbor is available in static, first check that the occupancy grid of the neighbor is not empty
                    if self.static_obstacles == [] or self.connectivity_matrix[i * self.n_grid + j, i_index * self.n_grid + j_index]:
                        neighbours.append([i_index, j_index])
        return neighbours

    def get_successors(self, position, time, speed, on_the_ground=False, debug=False):
        if self.occupancy_grid is None:
            self.initialize_occupancy_grid()
        successors = []
        if on_the_ground:
            end_t = float('inf')
            start_t = time
            i, j = self.get_coordinates(position)
            for interval in self.occupancy_grid[i, j]:
                if interval[0] < end_t and interval[1] > start_t:
                    intersection = [max(interval[0], start_t), min(interval[1], end_t)]
                    first_arrival_time = intersection[0]
                    successors.append([position, interval, first_arrival_time])
        else:
            neighbours = self.get_neighbours(position)
            origin = np.copy(position)
            interval_departure = self.get_interval(position, time)
            if interval_departure is None:
                print("There is an issue get_successors was called but at the position and time specified there is no free interval (i.e. it's already occupied)")
                print('This should not have happened as the position should have been unreachable')
                print(position)
                print(time)
                i, j = self.get_coordinates(position)
                intervals = self.occupancy_grid[i, j]
                print(i)
                print(j)
                print(intervals)
                return successors
            for neighbour in neighbours:
                destination = self.get_position(neighbour[0], neighbour[1])
                travel_time = np.linalg.norm(destination - origin) / speed
                start_t = time + travel_time
                end_t = interval_departure[1] + travel_time
                for interval in self.occupancy_grid[neighbour[0], neighbour[1]]:
                    if interval[0] < end_t and interval[1] > start_t:
                        intersection = [max(interval[0], start_t), min(interval[1], end_t)]
                        first_arrival_time = self.get_first_arrival_time(speed, origin, destination, intersection[0], intersection[1], debug=debug)
                        if first_arrival_time is not None:
                            successors.append([destination, interval, first_arrival_time])
        return successors

    def update_interval(self, interval, l, c):
        """ interval is an occupied interval, the function updates the list of free intervals when this function is
        called"""
        if l < 0 or l > self.n_grid - 1 or c < 0 or c > self.n_grid - 1:
            # The position is not on the grid
            return
        start_occupied = interval[0]
        end_occupied = interval[1]
        keep_iterating = True
        i = 0
        free_intervals = self.occupancy_grid[l, c]
        n_intervals = len(free_intervals)
        while keep_iterating:
            free_interval = free_intervals[i]
            if len(free_interval) == 0:
                # The interval is always occupied (static obstacle)
                break
            start = free_interval[0]
            end = free_interval[1]
            # If the intervals overlap
            if end_occupied > start and start_occupied < end:
                # the occupied interval is entirely included in the free interval
                if start_occupied > start and end_occupied < end:
                    free_intervals.pop(i)
                    free_intervals.insert(i, [start, start_occupied])
                    free_intervals.insert(i + 1, [end_occupied, end])
                    keep_iterating = False
                # the occupied interval entirely covers the free interval
                elif start_occupied <= start and end_occupied >= end:
                    free_intervals.pop(i)
                    n_intervals -= 1
                # occupied interval at the start of free interval
                elif start_occupied <= start:
                    free_interval[0] = end_occupied
                    i += 1
                elif end_occupied >= end:
                    free_interval[1] = start_occupied
                    i += 1
            else:
                i += 1
            if i >= n_intervals:
                keep_iterating = False

    def get_first_arrival_time(self, v, origin, destination, start_time, end_time, on_the_ground=False, debug=False):
        # Start time and end time define the interval during which the agent can arrive at destination
        # is_path_available, returns the info of the intruder agent that is a problem if the ownship was to depart at departure_time
        # If the aircraft was delayed more than max_delay it would arrive outside of the interval or depart outside of
        # its origin interval
        max_delay = end_time - start_time
        travel_time = np.linalg.norm(origin - destination) / v
        earliest_arrival_time = start_time
        departure_time = earliest_arrival_time - travel_time
        available, intruder_trajectory = self.is_path_available(departure_time, origin, destination, earliest_arrival_time, debug=debug)
        count = 0
        P_o = origin
        V_o = v * (destination - origin) / np.linalg.norm(destination - origin)
        while earliest_arrival_time < end_time and not available:
            count += 1
            # Set position of everything at departure time
            V_i = (intruder_trajectory[0][1] - intruder_trajectory[0][0]) / (intruder_trajectory[1][1] - intruder_trajectory[1][0])
            delta_t = intruder_trajectory[1][0] - departure_time
            P_i = intruder_trajectory[0][0] - delta_t * V_i
            # There is a conflict between the ownship and the intruder on the segment that was returned.
            # Note that when computing the required delay we are assuming the intruder started at P_i which is not true
            delay = self.get_time_to_leave(P_o, V_o, P_i, V_i, max_delay, on_the_ground=on_the_ground)
            if delay is None:
                return None
            if departure_time + delay < intruder_trajectory[1][1]:
                # if departure_time+delay<intruder_trajectory[1][0]:
                #     print('departure time '+str(departure_time)+', delay '+str(delay)+', intruder departure time '+str(intruder_trajectory[1][0]))
                #     print('Centralized manager, get_first_arrival time: delay too small, this is unexpected and will result in issues')
                # if debug:
                #     print('found valid delay on intruder time')
                earliest_arrival_time = earliest_arrival_time + delay
            else:
                # The intruder turns before getting away
                # departure_time is at least the time at the end of the intruder trajectory
                earliest_arrival_time = intruder_trajectory[1][1] + travel_time
            # Is the delay enough to avoid the intruder or are there other intruders?
            if count > 100:
                count = 0
                debug = True
                print('found the culprit infinite loop')
                print('earliest time ' + str(earliest_arrival_time))
                print('Maximum delay is ' + str(max_delay))
                print('latest arrival time is ' + str(max_delay + start_time))
                print('delay ' + str(delay))
                print('The agent wants to go from ' + str(origin) + ' to ' + str(destination))
                print('The agent is on the ground ' + str(on_the_ground))
                print('It is conflicting with the flight plan ' + str(intruder_trajectory))
                delay = self.get_time_to_leave(P_o, V_o, P_i, V_i, max_delay, on_the_ground=on_the_ground, debug=debug)
                print('P_0' + str(P_o))
                print('V_0' + str(V_o))
                print('P_i' + str(P_i))
                print('V_i' + str(V_i))

            departure_time = earliest_arrival_time - travel_time
            available, intruder_trajectory = self.is_path_available(departure_time, origin, destination, earliest_arrival_time, debug=debug)
        if available:
            return earliest_arrival_time
        else:
            return None  # No delay solved the issue

    def get_time_to_leave(self, P_o, V_o, P_i, V_i, max_time, on_the_ground=False, debug=False):
        # returns None if a conflict is unavoidable between time=0 and max_time
        # returns the value of a delay that allows to avoid the conflict
        # returns max_time
        delta_V = V_i - V_o
        delta_P = P_i - P_o
        delta_V_squared = np.dot(delta_V, delta_V)
        # Initial distance between ownship and intruder
        d_initial = np.linalg.norm(P_o - P_i)
        # Time of closest approach if the ownship does not move
        V_i_squared = np.dot(V_i, V_i)
        # To avoid numerical issues, make the avoidance area slightly bigger
        k_factor = 1.0001
        if V_i_squared == 0:
            # If the intruder is not moving there's no point in having a delay
            # This function is only called if there is a conflict on the time segment
            # Wait in place until the end of the segment
            t_cpa_stop = 0
            d_cpa_stop = d_initial
            if d_initial < self.minimum_separation_distance:
                return None
            else:
                return max_time
        else:
            t_cpa_stop = - np.dot(- V_i, P_o - P_i) / np.dot(V_i, V_i)
            d_cpa_stop = np.linalg.norm(P_i + t_cpa_stop * V_i - P_o)
        # Time of closest approach on the segment if the ownship does not move
        clamped_t_cpa_stop = clamp(0, max_time, t_cpa_stop)
        # Closest distance between the two agents on the segment if the ownship does not move (greater than d_cpa_stop)
        closest_distance_stop = np.linalg.norm(P_i + clamped_t_cpa_stop * V_i - P_o)
        if delta_V_squared == 0:
            a = -1
            Delta = -1
        else:
            # Time of closest approach if the ownship moves without delay
            t_cpa_no_delay = -np.dot(delta_V, delta_P) / delta_V_squared
            d_cpa_no_delay = np.linalg.norm(delta_P + t_cpa_no_delay * delta_V)
            Gamma = delta_P - delta_V * np.dot(delta_P, delta_V / delta_V_squared)
            Alpha = V_o - delta_V * (np.dot(delta_V, V_o) / delta_V_squared)
            c = np.dot(Gamma, Gamma) - (k_factor * self.minimum_separation_distance) ** 2
            b = 2 * np.dot(Alpha, Gamma)
            a = np.dot(Alpha, Alpha)
            Delta = b ** 2 - 4 * a * c
        if on_the_ground:
            if Delta >= 0 and a != 0:
                delay1 = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                delay2 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                if delay1 >= 0:
                    t_cpa_with_delay = -np.dot(delta_P + V_o * delay1, delta_V) / delta_V_squared
                    if delay1 <= t_cpa_with_delay:
                        return min(math.ceil(delay1 * 10) / 10, max_time)
                    else:
                        print('delay 1 greater than the time of closest approach found, no solution')
                        return None
                else:
                    if delay2 < 0:
                        print('This should not happen if there is a conflict, delay2<0')
                    else:
                        t_cpa_with_delay = -np.dot(delta_P + V_o * delay2, delta_V) / delta_V_squared
                        if delay2 < t_cpa_with_delay:
                            return min(math.ceil(delay2 * 10) / 10, max_time)
                        else:
                            a = np.dot(V_i, V_i)
                            b = 2 * np.dot(delta_P, V_i)
                            c = np.dot(delta_P, delta_P) - (k_factor * self.minimum_separation_distance) ** 2
                            det = b ** 2 - 4 * a * c
                            if det >= 0:
                                delay3 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                                return min(math.ceil(delay3 * 10) / 10, max_time)
                            else:
                                print('det<0, this should not happen')
                            return None
            else:
                # There is no delay that makes the closest point of approach be at that specific distance (i.e. if the agent was in the air there would be a conflict)
                # Since you're on the ground find when the intruder will be far enough that you can take off
                # TODO handle case when V_i is 0
                a = np.dot(V_i, V_i)
                b = 2 * np.dot(delta_P, V_i)
                c = np.dot(delta_P, delta_P) - (k_factor * self.minimum_separation_distance) ** 2
                det = b ** 2 - 4 * a * c
                if det >= 0:
                    delay3 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                    return min(math.ceil(delay3 * 10) / 10, max_time)
                else:
                    print('det<0, this should not happen')
                return None
        else:
            # in the air
            if debug:
                print('Delta is ' + str(Delta))
                if delta_V_squared != 0:
                    print('a is ' + str(a))
                    print('b is ' + str(b))
                    print('c is ' + str(c))
            if Delta >= 0 and a != 0:
                delay1 = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                delay2 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                if delay1 >= 0:
                    if debug:
                        print('looking at delay 1')
                    # The equation is only valid if the delay is smaller than the time of closest approach found on the next line
                    t_cpa_with_delay = -np.dot(delta_P + V_o * delay1, delta_V) / delta_V_squared
                    if delay1 <= t_cpa_with_delay:
                        if (0 <= t_cpa_stop < delay1 and d_cpa_stop < self.minimum_separation_distance) or d_initial < self.minimum_separation_distance:
                            # Sanity check
                            print('Cannot escape the conflict')
                            return None
                        else:
                            return min(math.ceil(delay1 * 10) / 10, max_time)
                    else:
                        print('delay 1 greater than the time of closest approach found, no solution')
                        return None
                else:
                    if delay2 < 0:
                        print('This should not happen if there is a conflict, delay2<0')
                    else:
                        if debug:
                            print('looking at delay 2')
                        # The equation is only valid if the delay is smaller than the time of closest approach found on the next line
                        t_cpa_with_delay = -np.dot(delta_P + V_o * delay2, delta_V) / delta_V_squared
                        if delay2 <= t_cpa_with_delay:
                            if (0 <= t_cpa_stop < delay2 and d_cpa_stop < self.minimum_separation_distance) or d_initial < self.minimum_separation_distance:
                                return None
                            else:
                                return min(math.ceil(delay2 * 10) / 10, max_time)
                        else:
                            # The solution delay2 is invalid
                            if closest_distance_stop <= self.minimum_separation_distance:
                                return None
                            else:
                                # Saved by the gong. Moving without delay will cause a conflict on the time segment.
                                # Waiting it out might not save you if the intruder keeps the same velocity, but if the intruder turns you have a chance
                                # Conflict is avoided on that time segment by waiting
                                return max_time
            else:
                # There is no delay that makes the closest point of approach be at that specific distance
                # This means that even by staying in place the two agents come close
                if closest_distance_stop < self.minimum_separation_distance:
                    return None
                else:
                    # Saved by the gong
                    return max_time

    def quadratic(self, start_point, end_point, cell_point):
        # Given a segment going from start_point to end_point, find the points between which the segment is closest
        # than the minimum separation distance to the cell point
        # returns the interval expressed in term of fraction (between 0 and 1)
        interval = []
        length = np.linalg.norm(end_point - start_point)
        side = start_point - cell_point
        if length == 0:
            # Agent is staying in place
            if np.linalg.norm(side) <= self.minimum_separation_distance:
                interval = [0, 1]
        else:
            u = (end_point - start_point) / length
            b = np.dot(u, side)
            # Adding slightly superior to 1 k-factor for numerical issues (i.e. make the interval slightly bigger)
            c = np.dot(side, side) - (1.0001 * self.minimum_separation_distance) ** 2
            delta = b ** 2 - c
            if delta > 0:
                x1 = -b - math.sqrt(delta)
                x2 = -b + math.sqrt(delta)
                x1 = clamp(0, length, x1)
                x2 = clamp(0, length, x2)
                interval = [x1 / length, x2 / length]
        return interval

    def get_interval(self, position, time):
        i, j = self.get_coordinates(position)
        intervals = self.occupancy_grid[i, j]
        for interval in intervals:
            if interval[0] <= time <= interval[1]:
                return interval
        return None

    def get_coordinates(self, position):
        i = int(round(position[0] / self.cell_length))
        j = int(round(position[1] / self.cell_length))
        return i, j

    def get_position(self, i, j):
        x = i * self.cell_length
        y = j * self.cell_length
        return np.array([x, y])

    def is_path_available(self, ownship_start_time, start_point, end_point, end_time, debug=False):
        """ Assuming the ownship leaves the start_point at the start_time to finish at end_point and end_time, is there a conflict with other
        flight plans ?
        Returns available (Bool), intruder trajectory
        Returns only the section of the intruder path on which the collision will occur => the time interval returned is included in [ownship_start_time, end_time]"""
        # Naive approach
        # Iterate over all flight plans, find all flight segments that overlap in time with the input time
        # Compute for each of those segments the closest time of approach
        # Need to intersect time between time_start and time_end and get the resulting trajectory
        ownship_velocity = (end_point - start_point) / (end_time - ownship_start_time)
        for agent_id in self.flight_plans:
            flight_plan = self.flight_plans[agent_id]
            trajectory, times = flight_plan.get_planned_trajectory_between(ownship_start_time, end_time)
            if times is not None and len(times) >= 2 and abs(times[0] - times[1]) < 0.0001:
                print('there is an issue with get planned trajectory between')
                print(trajectory)
                print(times)
                flight_plan.get_planned_trajectory_between(ownship_start_time, end_time, debug=True)
            if trajectory is not None:
                n = len(trajectory)
                intruder_segment_start_pos = np.copy(trajectory[0])
                intruder_segment_start_time = times[0]
                analysis_start_time = max(ownship_start_time, intruder_segment_start_time)
                ownship_pos = start_point + (analysis_start_time - ownship_start_time) * ownship_velocity
                for i in range(1, n):
                    intruder_segment_end_pos = trajectory[i]
                    intruder_segment_end_time = times[i]
                    intruder_velocity = (intruder_segment_end_pos - intruder_segment_start_pos) / (intruder_segment_end_time - intruder_segment_start_time)
                    intruder_pos = intruder_segment_start_pos + (analysis_start_time - intruder_segment_start_time) * intruder_velocity
                    # Let's compute the closest time of approach
                    delta_v = intruder_velocity - ownship_velocity
                    delta_v_squared = np.dot(delta_v, delta_v)
                    delta_p = intruder_pos - ownship_pos
                    if delta_v_squared != 0:
                        t_cpa = analysis_start_time - np.dot(delta_v, delta_p) / delta_v_squared
                        # Let's find the time of closest approach on the segment
                        clamped_t_cpa = clamp(analysis_start_time, intruder_segment_end_time, t_cpa) - analysis_start_time  # between 0 and the end of the segment (t_elapsed)
                    else:
                        # If ownship and intruder have the same velocity vector then the distance between them is constant
                        clamped_t_cpa = 0
                    closest = np.linalg.norm(intruder_pos + intruder_velocity * clamped_t_cpa - ownship_pos - ownship_velocity * clamped_t_cpa)
                    if closest < self.minimum_separation_distance:
                        if debug and abs(times[i - 1] - times[i]) < 0.00001:
                            print('Problem with time')
                        if debug:
                            trajectory, times = flight_plan.get_planned_trajectory_between(ownship_start_time, end_time, debug=True)
                        return False, [trajectory[i - 1:i + 1], times[i - 1:i + 1]]
                    intruder_segment_start_time = intruder_segment_end_time
                    intruder_segment_start_pos = intruder_segment_end_pos
                    analysis_start_time = max(ownship_start_time, intruder_segment_start_time)
                    ownship_pos = start_point + (analysis_start_time - ownship_start_time) * ownship_velocity

        return True, None

    def add_flight_plan(self, flight_plan, my_id, agent, my_time, priority=False):
        perturbed_agents_in_air = []
        perturbed_agents_ground = []
        if priority:
            # The new flight plan has priority over all flight plans that do not already have priority
            # For now priority agents have constant velocity this can be used to determine closest point of approach without having to iterate on the priority agent flight plan
            # Iterate over all flight plans
            for agent_id in self.flight_plans:
                if self.agents[agent_id].agent_logic != 'dumb':
                    # retrieve the trajectory between the two times
                    trajectory, times = self.flight_plans[agent_id].get_planned_trajectory_between(flight_plan.times[0], flight_plan.times[-1])
                    if times is not None:
                        for i in range(0, len(times) - 1):
                            priority_position, priority_velocity = flight_plan.get_planned_position_at(times[i], return_velocity=True)
                            vel = (trajectory[i + 1] - trajectory[i]) / (times[i + 1] - times[i])
                            delta_p = trajectory[i] - priority_position
                            delta_v = vel - priority_velocity
                            delta_v_squared = np.dot(delta_v, delta_v)
                            if delta_v_squared != 0:
                                t_cpa = - np.dot(delta_p, delta_v) / delta_v_squared
                            else:
                                t_cpa = 0
                            clamped_t_cpa = clamp(0, times[i + 1] - times[i], t_cpa)
                            closest = np.linalg.norm(delta_p + clamped_t_cpa * delta_v)
                            if closest < self.minimum_separation_distance:
                                # Keep a list of agents that will have to replan or yield priority
                                if self.flight_plans[agent_id].times[0] > my_time:
                                    perturbed_agents_ground.append(agent_id)
                                else:
                                    perturbed_agents_in_air.append(agent_id)
                                break
        for agent_id in perturbed_agents_in_air + perturbed_agents_ground:
            del self.flight_plans[agent_id]
        self.flight_plans[my_id] = flight_plan
        self.agents[my_id] = agent
        # Reset planning
        if priority:
            self.reset_occupancy_grid()
        else:
            self.update_availability_grid(flight_plan)
        # Make perturbed agents replan or switch behavior, starting with those in the air
        for agent_id in perturbed_agents_in_air + perturbed_agents_ground:
            new_flight_plan = self.agents[agent_id].yield_priority(flight_plan, my_time)
            if new_flight_plan is not None:
                self.flight_plans[agent_id] = new_flight_plan

    def terminate_flight_plan(self, id):
        if id in self.flight_plans:
            del self.flight_plans[id]
        if id in self.agents:
            del self.agents[id]
        else:
            print('Error in centralized manager, the id should be in the agent list')

    def update_availability_grid(self, flight_plan):
        old_pos = flight_plan.positions[0]
        old_time = flight_plan.times[0]
        for i in range(1, len(flight_plan.positions)):
            # The cells are in the grid coordinates, the flight plan is in world coordinates
            new_pos = flight_plan.positions[i]
            cells = self.get_cells_along_supercover(old_pos, new_pos)
            new_time = flight_plan.times[i]
            dt = new_time - old_time
            for cell in cells:
                # Find the times where the cell is occupied by solving the quadratic equation
                cell_position = self.get_position(cell[0], cell[1])
                interval = self.quadratic(old_pos, new_pos, cell_position)
                if interval != []:
                    time_interval = [old_time + dt * interval[0],
                                     old_time + dt * interval[1]]
                    self.update_interval(time_interval, cell[0], cell[1])
            old_pos = new_pos
            old_time = flight_plan.times[i]

    def on_the_grid_get_cells_along(self, position_a, position_b):
        """ Simplified algo for SIPP when all agents are restricted to be on grid points, always traveling between neighboring points, returns all cells when traveling diagonally """
        cells = []
        x_a = int(round(position_a[0] / self.cell_length))
        y_a = int(round(position_a[1] / self.cell_length))
        x_b = int(round(position_b[0] / self.cell_length))
        y_b = int(round(position_b[1] / self.cell_length))
        cells.append([x_a, y_a])
        cells.append([x_b, y_b])
        if x_a != x_b and y_a != y_b:
            cells.append([x_a, y_b])
            cells.append([x_b, y_a])
        return cells

    def get_cells_along_supercover(self, position_a, position_b):
        cells = set()
        x_a = position_a[0] / self.cell_length
        y_a = position_a[1] / self.cell_length
        x_b = position_b[0] / self.cell_length
        y_b = position_b[1] / self.cell_length
        final_x = int(round(x_b))
        final_y = int(round(y_b))
        dx = x_b - x_a
        dy = y_b - y_a
        if dx == 0:
            step_x = 0
            if dy == 0:
                step_y = 0
            elif dy > 0:
                step_y = 1
            else:
                step_y = -1
        else:
            if dx > 0:
                step_x = 1
            else:
                step_x = -1
            step_y = dy / dx * step_x
        x = x_a
        y = y_a
        reached = False
        while not reached:
            int_x = int(round(x))
            int_y = int(round(y))
            n_x = [-1, 0, 1]
            n_y = [-1, 0, 1]
            for i in n_x:
                for j in n_y:
                    cells.add((int_x + i, int_y + j))
            if dx * (final_x - int_x) <= 0 and dy * (final_y - int_y) <= 0:
                reached = True
            x = x + step_x
            y = y + step_y
        return cells

    def get_cells_along(self, position_a, position_b):
        """ DDA (Digital Differential Analyzer) line algorithm. Implemented using an incremental error algorithm
        (with floats), to account for the fact that the trajectory does not snap on the grid
        Supercover version of the algorithm (returns all the grid cell that the line encounter and not just one per axis)
        Different sources used:
        http://eugen.dedu.free.fr/projects/bresenham/ (supercover version of Bresenham)
        http://groups.csail.mit.edu/graphics/classes/6.837/F01/Lecture04/lecture04.pdf (logic behind the algorithm)
        https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html (good step by step walkthrough of how the
        algorithm is modified to use integer arithmetic)
        Because of the floating aspect the code is not optimized (but it could be)
        Took way too long to implement because of dumb sign errors I made
        """
        cells = []
        x_a = position_a[0] / self.cell_length
        y_a = position_a[1] / self.cell_length
        x_b = position_b[0] / self.cell_length
        y_b = position_b[1] / self.cell_length
        dx = x_b - x_a
        dy = y_b - y_a
        if dy < 0:
            y_step = -1
            dy = -dy
        else:
            y_step = 1
        if dx < 0:
            x_step = -1
            dx = -dx
        else:
            x_step = 1
        x = x_a
        y = y_a
        if dx >= dy:
            if dx != 0:
                m = (dy / dx)
                y = y_a + m * (round(x_a) - x_a) * x_step * y_step
                eps = y - round(y)
            else:
                # dx==0, since dx>=dy, dy=0
                cells.append([int(round(x)), int(round(y))])
                return cells
            x = round(x_a)
            y = y_a + m * (x - x_a) * x_step * y_step
            cells.append([int(round(x)), int(round(y))])
            error_prev = eps
            error = eps * y_step
            while -0.5 > x - round(x_b) or x - round(x_b) > 0.5:
                x += x_step
                if (error + m) > 0.5:
                    y += y_step
                    error += m - 1
                    if (error_prev + m / 2) < 0.5:
                        cells.append([int(round(x)), int(round(y - y_step))])
                    elif (error_prev + m / 2) > 0.5:
                        cells.append([int(round(x - x_step)), int(round(y))])
                    else:
                        cells.append([int(round(x - x_step)), int(round(y))])
                        cells.append([int(round(x)), int(round(y - y_step))])
                else:
                    error += m
                cells.append([int(round(x)), int(round(y))])
                error_prev = error
        else:
            if dy != 0:
                m = dx / dy
                x = x_a + m * (round(y_a) - y_a) * x_step * y_step
                eps = x - round(x)
            y = round(y_a)
            x = x_a + m * (y - y_a) * x_step * y_step
            cells.append([int(round(x)), int(round(y))])
            error_prev = eps
            error = eps * x_step
            while -0.5 > y - round(y_b) or y - round(y_b) > 0.5:
                y += y_step
                if (error + m) > 0.5:
                    x += x_step
                    error += m - 1
                    if (error_prev + m / 2) < 0.5:
                        cells.append([int(round(x - x_step)), int(round(y))])
                    elif (error_prev + m / 2) > 0.5:
                        cells.append([int(round(x)), int(round(y - y_step))])
                    else:
                        cells.append([int(round(x - x_step)), int(round(y))])
                        cells.append([int(round(x)), int(round(y - y_step))])
                else:
                    error += m
                cells.append([int(round(x)), int(round(y))])
                error_prev = error
        return cells

    def validate_flight_plan(self, flight_plan):
        old_pos = flight_plan.positions[0]
        old_t = flight_plan.times[0]
        for i in range(1, len(flight_plan.positions)):
            pos = flight_plan.positions[i]
            t = flight_plan.times[i]
            if not self.is_path_available(old_t, old_pos, pos, t):
                return False
            old_pos = pos
            old_t = t
        return True
