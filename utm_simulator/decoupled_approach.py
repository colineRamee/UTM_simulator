import numpy as np
import math
import heapq
from utm_simulator.my_utils import get_trajectory_circle_intersection


class DecoupledApproach:
    def __init__(self, start, goal, start_time, speed, centralized_manager, tolerance):
        self.centralized_manager = centralized_manager
        self.speed = speed
        self.start = start
        self.goal = goal
        self.start_time = start_time
        self.tolerance = tolerance

    def search(self, debug=False):
        # Energy consideration. Between two nodes that have the same cost_to_go (i.e. that can be reached at the same time) and the same heuristic (i.e. same position)
        path = self.centralized_manager.get_path_smoothed(self.start, self.goal)
        graph = Line(path, self.speed, self.centralized_manager.minimum_separation_distance, self.start_time)
        start_node = graph.get_node(type='start')
        end_node = graph.get_node(type='end')
        start_node_t = start_node.get_node_at_time(self.start_time)
        open_queue = PriorityQueue()
        open_cost_so_far = {}
        open_energy_so_far = {}
        came_from = {}
        h = graph.get_heuristic(start_node_t)
        open_queue.push(start_node_t, h, h, h)
        open_cost_so_far[start_node_t] = 0
        open_energy_so_far[start_node_t] = 0  # energy = time in the air
        came_from[start_node_t] = None
        success = False
        plan = []
        times = []
        while not open_queue.empty():
            current = open_queue.pop()
            # Condition for success: within tolerance of goal and not at the start node
            if current.node != start_node and np.linalg.norm(current.node.position - end_node.position) <= self.tolerance:
                success = True
                break
            for neighbor in graph.get_neighbors_dynamic(current):
                on_the_ground = current.node == start_node and neighbor.node == start_node
                path_available = False
                if not on_the_ground:
                    path = graph.get_path(current.node, neighbor.node)
                    if len(path) == 2:
                        path_available = self.centralized_manager.is_path_available(current.time, current.node.position, neighbor.node.position, neighbor.time)
                    else:
                        start_pos = path[0]
                        start_time = current.time
                        for i in range(1, len(path)):
                            travel_time = np.linalg.norm(path[i] - start_pos) / self.speed
                            path_available = self.centralized_manager.is_path_available(start_time, start_pos, path[i], start_time + travel_time)
                            start_pos = path[i]
                            start_time += travel_time
                            if not path_available:
                                break
                if on_the_ground or path_available:
                    cost_to_go = graph.get_travel_cost(current, neighbor)
                    if on_the_ground:
                        energy_to_go = 0
                    else:
                        energy_to_go = cost_to_go
                    new_cost = open_cost_so_far[current] + cost_to_go
                    new_energy = open_energy_so_far[current] + energy_to_go
                    # To favor waiting on the ground versus waiting in the air, all other things being equal
                    if neighbor not in open_cost_so_far or open_cost_so_far[neighbor] > new_cost or (open_cost_so_far[neighbor] == new_cost and open_energy_so_far[neighbor] > new_energy):
                        open_cost_so_far[neighbor] = new_cost
                        open_energy_so_far[neighbor] = new_energy
                        came_from[neighbor] = current
                        open_queue.push(neighbor, new_cost + neighbor.node.heuristic, new_energy + neighbor.node.heuristic, neighbor.node.heuristic)
        if success:
            # Handle issues that arise due to having a tolerance
            # The last node is within the tolerance distance from the goal, it might actually be the goal or it might be right at tolerance distance from the goal, we want the trajectory to stop at exactly 0.95*tolerance from the goal
            # This ensure that the agent will get deleted
            if debug:
                previous = came_from[current]
                print('Plan returned by A* positions: ' + str(previous.node.position) + ' ' + str(current.node.position))
                print('Plan returned by A* times: ' + str(previous.time) + ' ' + str(current.time))
                print('Goal ' + str(end_node.position))
            # Need to find the intersection between the trajectory and 0.95 tolerance circle around the goal
            # Cases to consider: 1. current is at greater than 0.95 tolerance from the goal, 2. current is within 0.95 from the goal
            final_distance = np.linalg.norm(current.node.position - end_node.position)
            if final_distance < 0.95 * self.tolerance:
                # Find intersection of path between previous and current and the 0.95tolerance
                # Note that we know that previous is not within tolerance, so we know that intersection is on this part of the trajectory
                if debug:
                    print('final distance is within 0.95*tolerance, shorten the path')
                previous = came_from[current]
                path = graph.get_path(previous.node, current.node)
                if debug:
                    print('Path between previous and current ' + str(path))
                path_time = [previous.time]
                index = 0
                for i in range(1, len(path)):
                    distance = np.linalg.norm(path[i] - path[i - 1])
                    path_time.append(path_time[i - 1] + distance / self.speed)
                    if np.linalg.norm(path[i] - end_node.position) <= 0.95 * self.tolerance:
                        index = i
                        break
                if debug:
                    print('Index of path point within tolerance ' + str(index))
                t_intersection, pos_intersection = get_trajectory_circle_intersection(end_node.position, 0.95 * self.tolerance, path[index - 1], self.speed * (path[index] - path[index - 1]) / np.linalg.norm(path[index] - path[index - 1]))
                if debug:
                    print('Intersection of trajectory with tolerance, time ' + str(t_intersection) + ', pos ' + str(pos_intersection))
                if len(t_intersection) == 1:
                    # This should not happen (path tangent to 0.95tol circle)
                    plan.append(pos_intersection[0])
                    times.append(path_time[index - 1] + t_intersection[0])
                else:
                    if 0 <= t_intersection[0] <= path_time[index] - path_time[index - 1]:
                        # Valid intersection time
                        times.append(path_time[index - 1] + t_intersection[0])
                        plan.append(pos_intersection[0])
                    elif 0 <= t_intersection[1] <= path_time[index] - path_time[index - 1]:
                        times.append(path_time[index - 1] + t_intersection[1])
                        plan.append(pos_intersection[1])
                    else:
                        print('Error in decoupled approach tolerance handling')
                for i in range(index - 1, 0, -1):
                    times.append(path_time[i])
                    plan.append(path[i])
                times.append(previous.time)
                plan.append(previous.node.position)
            else:
                # tolerance >= final_distance >= 0.95* tolerance
                if debug:
                    print('final distance is not within 0.95 of the goal, increase path')
                position = current.node.position + (final_distance - 0.95 * self.tolerance) * (end_node.position - current.node.position) / final_distance
                time = current.time + (final_distance - 0.95 * self.tolerance) / self.speed
                plan.append(position)
                times.append(time)
                plan.append(current.node.position)
                times.append(current.time)
                previous = current
            stop = False
            parent = came_from[previous]
            if previous.node == start_node:
                stop = True
            while parent is not None and (not stop):
                path = graph.get_path(previous.node, parent.node)
                previous_time = previous.time
                for i in range(1, len(path) - 1):
                    plan.append(path[i])
                    travel_time = np.linalg.norm(path[i] - path[i - 1]) / self.speed
                    previous_time = previous_time - travel_time
                    times.append(previous_time)
                plan.append(parent.node.position)
                times.append(parent.time)
                if parent.node == start_node:
                    # Handle ground delays, the trajectory should start when the agent takes off
                    stop = True
                previous = parent
                parent = came_from[parent]
        plan.reverse()
        times.reverse()
        return success, plan, times


class Line:
    def __init__(self, path, velocity, maximum_cell_size, start_time):
        self.path = path
        self.velocity = velocity
        self.maximum_cell_size = maximum_cell_size
        self.start_time = start_time
        self.nodes, self.junctions = self.initialize_line()
        self.n_nodes = len(self.nodes)

    def initialize_line(self):
        epsilon = 1e-3
        path_segments_length = []
        remaining_distance_on_path = [0]
        for k in range(len(self.path) - 1, 0, -1):
            length = np.linalg.norm(self.path[k] - self.path[k - 1])
            path_segments_length.append(length)
            remaining_distance_on_path.append(remaining_distance_on_path[-1] + length)
        remaining_distance_on_path.reverse()
        path_segments_length.reverse()
        total_path_length = remaining_distance_on_path[0]
        n = math.ceil(total_path_length / self.maximum_cell_size)
        self.step = total_path_length / n
        self.time_step = self.step / self.velocity

        junctions = [[] for i in range(0, n - 1)]

        position = self.path[0]
        direction = (self.path[1] - self.path[0]) / path_segments_length[0]
        node_index = 0
        path_index = 0
        nodes = [Node(self.path[path_index], node_index, total_path_length / self.velocity, self.convert_time)]
        distance_on_segment = 0
        distance_remaining_before_sample = self.step
        while node_index != n:
            if distance_on_segment + distance_remaining_before_sample >= path_segments_length[path_index]:
                distance_remaining_before_sample = distance_on_segment + distance_remaining_before_sample - path_segments_length[path_index]
                if distance_remaining_before_sample > epsilon:
                    junctions[node_index].append(self.path[path_index + 1])
                else:
                    node_index += 1
                    nodes.append(Node(self.path[path_index + 1], node_index, remaining_distance_on_path[path_index + 1] / self.velocity, self.convert_time))
                    distance_remaining_before_sample = self.step
                # Moving on to next path segment
                distance_on_segment = 0
                path_index += 1
                if path_index == len(self.path) - 1:
                    # This should only happen at the last node
                    position = None
                    direction = None
                else:
                    position = self.path[path_index]
                    direction = (self.path[path_index + 1] - self.path[path_index]) / path_segments_length[path_index]
            else:
                position = position + distance_remaining_before_sample * direction
                distance_on_segment += distance_remaining_before_sample
                distance_remaining_before_sample = self.step
                heuristic = (remaining_distance_on_path[path_index + 1] + path_segments_length[path_index] - distance_on_segment) / self.velocity
                node_index += 1
                nodes.append(Node(position, node_index, heuristic, self.convert_time))
        return nodes, junctions

    def get_node(self, type=''):
        if type == 'start':
            return self.nodes[0]
        elif type == 'end':
            return self.nodes[-1]
        else:
            print('Line get_node this type of node cannot be requested ' + str(type))

    def get_neighbors_static(self, node):
        neighbors = []
        index = node.index
        if index > 0:
            neighbors.append(self.nodes[index - 1])
        if index < self.n_nodes - 1:
            neighbors.append(self.nodes[index + 1])
        return neighbors

    def get_travel_cost(self, node_a, node_b):
        return node_b.time - node_a.time

    def get_neighbors_dynamic(self, node_t):
        time_index = node_t.discrete_time
        neighbors_t = []
        neighbors = self.get_neighbors_static(node_t.node)
        for neighbor in neighbors:
            neighbor_t = neighbor.get_node_at_time(discrete_time=time_index + 1)
            neighbors_t.append(neighbor_t)
        hover_node = node_t.node.get_node_at_time(discrete_time=time_index + 1)
        neighbors_t.append(hover_node)
        return neighbors_t

    def get_path(self, node_a, node_b):
        start_index = node_a.index
        end_index = node_b.index
        if start_index == end_index:
            path = [node_a.position, node_b.position]
        else:
            path = [node_a.position]
            increment = np.sign(end_index - start_index)
            for i in range(start_index, end_index, increment):
                if increment == 1:
                    k = i
                elif increment == -1:
                    k = i - 1
                for junction in self.junctions[k]:
                    path.append(junction)
                path.append(self.nodes[i + increment].position)
        return path

    def get_heuristic(self, node_t):
        return node_t.node.heuristic

    def convert_time(self, actual_time=None, discrete_time=None):
        if actual_time is None:
            if discrete_time is None:
                print('Error, convert_time must be called with actual_time or discrete_time specified')
            return self.start_time + discrete_time * self.time_step
        else:
            return round((actual_time - self.start_time) / self.time_step)


class Node:
    def __init__(self, position, index, heuristic, convert_time):
        self.position = position
        self.index = index
        self.heuristic = heuristic
        self.convert_time = convert_time
        self.time_dic = {}

    def get_node_at_time(self, actual_time=None, discrete_time=None):
        if actual_time is None:
            actual_time = self.convert_time(discrete_time=discrete_time)
        else:
            discrete_time = self.convert_time(actual_time=actual_time)
        if discrete_time not in self.time_dic:
            self.time_dic[discrete_time] = FourDNode(self, actual_time, discrete_time)
        return self.time_dic[discrete_time]


class FourDNode:
    def __init__(self, node, time, discrete_time):
        self.node = node
        self.time = time
        self.discrete_time = discrete_time

    def __lt__(self, other):
        return self.node.index < other.node.index


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def push(self, state, priority1, priority2, priority3):
        heapq.heappush(self.elements, (priority1, priority2, priority3, state))

    def pop(self):
        while self.elements:
            priority, priority2, priority3, state = heapq.heappop(self.elements)
            return state
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return len(self.elements) == 0
