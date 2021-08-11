import numpy as np
import random
import math
import json
from utm_simulator import agent
from utm_simulator.my_utils import MyLogger
from utm_simulator import layer


class LayerEnvironment:
    def __init__(self, size_map, min_distance, agent_speed, sensing_radius, tolerance, utm_on, desired_number_of_agents=1, centralized_manager=None,
                 multiple_planning_agents=False, structure=None, demand=None, static_obstacles=None, priority_agents=0, simulation_type='reactive',
                 algo_type='A_star_8', log_type='short', log_density=False, n_valid_agents_for_simulation_end=None, logger=MyLogger(), n_layers=1):
        self.my_logger = logger
        self.min_distance = min_distance
        self.max_speed = agent_speed
        self.size_map = size_map
        self.tolerance = tolerance
        self.layers = []
        self.n_layers = n_layers
        self.default_planning_time_step = 2 * min_distance / agent_speed
        for i in range(0, n_layers):
            self.layers.append(layer.Layer(size_map, min_distance, agent_speed, sensing_radius, tolerance, utm_on, static_obstacles=static_obstacles, demand=demand,
                                           default_planning_time_step=self.default_planning_time_step, multiple_planning_agents=multiple_planning_agents,
                                           simulation_type=simulation_type, algo_type=algo_type, log_type=log_type, log_density=log_density, logger=self.my_logger))
        self.desired_number_of_agents = desired_number_of_agents

        self.centralized_manager = centralized_manager  # Just for generating start/end pairs

        self.multiple_planning_agents = multiple_planning_agents
        self.structure = structure
        # If structure is None layer assignment if n_layers>1 will be at random
        # If structure is not None, the number of layers must match the structure
        if structure is not None:
            if structure['type'] == 'layer':
                parameters = np.array(structure['parameters'])
                if len(parameters.shape) == 1:
                    n_parameters = 1
                else:
                    n_parameters = parameters.shape[0]
                if n_parameters != n_layers:
                    logger.log('The number of parameters for the layer structure should match the number of layers requested in the simulation')
            else:
                logger.log('The Layer environment is not setup to work with this structure type ' + str(structure))
        self.lz_coordinates = None

        self.demand = demand
        self.static_obstacles = static_obstacles

        self.n_priority_agents = priority_agents
        self.n_min_valid_agents = n_valid_agents_for_simulation_end
        self.n_valid_density_agents = 0

        self.n_controlled_agents_current = 0
        self.agent_added_time = 0
        self.priority_agent_added_time = 0
        self.total_number_of_agents = 0

        self.simulation_type = simulation_type
        self.algo_type = algo_type
        self.log_type = log_type
        self.log_density = log_density

        self.t_density = None
        self.t0 = None
        self.ramping_up = True
        if priority_agents != 0:
            self.ramping_up_priority = True
        else:
            self.ramping_up_priority = False

    def run(self, time, sim_dt):
        for layer in self.layers:
            layer.run(time, sim_dt)
        # Add agents (based on sim phase)
        self.add_agents(time + sim_dt)
        # Remove outsiders (and add agents to compensate agents that left)
        agents_to_add = []
        valid_agents = []
        found_old_agents_in_one_layer = False
        for layer in self.layers:
            found_old_agents, n_valid_agents_finished, agents_removed_to_compensate = layer.remove_outsiders(time + sim_dt, self.t_density, self.t0)
            agents_to_add = agents_to_add + agents_removed_to_compensate
            valid_agents.append(n_valid_agents_finished)
            if found_old_agents:
                found_old_agents_in_one_layer = True
        if self.t_density is not None and self.t0 is None and not found_old_agents_in_one_layer:
            self.t0 = time + sim_dt
        self.n_valid_density_agents += sum(valid_agents)
        self.add_agents(time + sim_dt, agent_types=agents_to_add)
        # Build KDTree and check loss of separation
        for layer in self.layers:
            layer.build_kd_tree()
            layer.check_collisions(time + sim_dt)

        # Checking and updating simulation status
        self.check_simulation_phase(time + sim_dt)
        if not self.multiple_planning_agents and self.n_controlled_agents_current == 0:
            finished = True
        elif self.n_min_valid_agents is not None and self.n_valid_density_agents >= self.n_min_valid_agents:
            finished = True
        else:
            finished = False
        # Add agents to the appropriate layer
        return finished

    def check_simulation_phase(self, time):
        controlled_agents = 0
        n_priority_agents = 0
        for layer in self.layers:
            controlled_agents += len(layer.smart_agents) + len(layer.waiting_agent)
            n_priority_agents += len(layer.dumb_agents)
        if self.ramping_up:
            if controlled_agents >= self.desired_number_of_agents:
                self.ramping_up = False
                self.t_density = time
        if self.ramping_up_priority:
            if n_priority_agents >= self.n_priority_agents:
                self.ramping_up_priority = False
        self.n_controlled_agents_current = controlled_agents

    def get_random_start_and_end(self, heading_range=None, protected_area_start=None):
        if self.demand is None:
            return self.get_random_start_and_end_on_edge(heading_range=heading_range, protected_area_start=protected_area_start)
        else:
            if self.demand['type'] == 'hub_and_spoke':
                return self.get_start_and_end_with_hub_and_spoke(heading_range=heading_range, protected_area_start=protected_area_start)
            elif self.demand['type'] == 'population_density':
                return self.get_start_and_end_with_pop_density(heading_range=heading_range, protected_area_start=protected_area_start)
            else:
                self.my_logger.log('This type of demand is not implemented ' + self.demand['type'])

    def get_random_start_and_end_outside(self):
        a = random.randint(1000, self.size_map - 1000)
        b = random.randint(1000, self.size_map - 1000)
        side_a = random.randint(0, 3)
        side_b = (side_a + random.randint(1, 3)) % 4
        if side_a % 2 == 0:
            start_x = a
            if side_a == 0:
                start_y = -500
            else:
                start_y = self.size_map + 500
        else:
            start_y = a
            if side_a == 1:
                start_x = self.size_map + 500
            else:
                start_x = -500
        if side_b % 2 == 0:
            end_x = b
            if side_b == 0:
                end_y = -500
            else:
                end_y = self.size_map + 500
        else:
            end_y = b
            if side_b == 1:
                end_x = self.size_map + 500
            else:
                end_x = -500
        return np.array([start_x, start_y]), np.array([end_x, end_y])

    def get_random_start_and_end_on_edge(self, heading_range=None, protected_area_start=None):
        if heading_range is None:
            valid_structure = True
        else:
            valid_structure = False
        valid_start_and_end = False
        n_trials_structure = 0
        max_trials_structure = 100
        while (not valid_structure or not valid_start_and_end) and n_trials_structure < max_trials_structure:
            a = random.randint(1, self.size_map - 1)  # +1, -1 to avoid corner cases
            side_a = random.randint(0, 3)
            # depending on the side picked build the start vector
            start_x = a * (-(side_a % 2) + 1) + (side_a == 1) * self.size_map
            start_y = a * (side_a % 2) + (side_a == 2) * self.size_map
            start = np.array([float(start_x), float(start_y)])
            if self.algo_type == 'SIPP':
                i, j = self.centralized_manager.get_coordinates(start)
                start = self.centralized_manager.get_position(i, j)
            # Check if there is an issue with a protected area
            if protected_area_start is not None:
                # This prevents a dumb agent from being initialized too close to a reactive agent
                number_of_trials = 0
                max_number_of_trials = 10
                while np.linalg.norm(protected_area_start['center'] - start) <= protected_area_start['radius'] and number_of_trials < max_number_of_trials:
                    a = random.randint(0, self.size_map)
                    # b = random.randint(0, self.size_map)
                    start_x = a * (-(side_a % 2) + 1) + (side_a == 1) * self.size_map
                    start_y = a * (side_a % 2) + (side_a == 2) * self.size_map
                    number_of_trials += 1
                    start = np.array([float(start_x), float(start_y)])
                if np.linalg.norm(protected_area_start['center'] - start) <= protected_area_start['radius']:
                    print(start)
                    print(protected_area_start['center'])
                    self.my_logger.log('getRandomStartAndEnd failed to place the random agent in a conflict free zone')
            max_number_of_trials = 10
            number_of_trials = 0
            valid_start_and_end = False
            while not valid_start_and_end and number_of_trials < max_number_of_trials:
                angle = random.random() * math.pi
                end_x = None
                end_y = None
                if angle == 0:  # probability 0
                    # Returns a corner of the line where it started
                    end_x = (side_a == 1) * self.size_map
                    end_y = (side_a == 2) * self.size_map
                elif angle == (math.pi / 2):
                    side_b = (side_a + 2) % 4
                    end_x = a * (-(side_b % 2) + 1) + (side_b == 1) * self.size_map
                    end_y = a * (side_b % 2) + (side_b == 2) * self.size_map
                else:
                    # compute the intersection with all three other sides (catch exception if angle is pi/2)
                    # Also exception if we are exactly at the corner
                    for i in range(1, 4):
                        side_b = (side_a + i) % 4
                        if (side_b % 2) == 1:
                            x = (side_b == 1) * self.size_map
                            y = start_y + math.tan(angle) * (x - start_x)
                            if 0 <= y <= self.size_map and x != start_x:
                                my_side = i
                                end_x = x
                                end_y = y
                        else:
                            y = (side_b == 2) * self.size_map
                            x = start_x + (1 / math.tan(angle)) * (y - start_y)
                            if 0 <= x <= self.size_map and y != start_y:
                                my_side = i
                                end_x = x
                                end_y = y
                if end_x is None or end_y is None:
                    print('environment random start and end bug')
                    print(angle)
                    print(start_x)
                    print(start_y)
                    print('side a, ', side_a)
                    print('side b, ', my_side)

                end = np.array([float(end_x), float(end_y)])
                if self.algo_type == 'SIPP':
                    i, j = self.centralized_manager.get_coordinates(end)
                    end = self.centralized_manager.get_position(i, j)
                # Is the pair valid ?
                if np.linalg.norm(end - start) > self.tolerance:
                    valid_start_and_end = True
                number_of_trials += 1
            if number_of_trials >= max_number_of_trials:
                print('get random start and end failed to find a valid pair')
            if heading_range is not None:
                heading = math.atan2(end[1] - start[1], end[0] - start[0]) * 180 / math.pi
                if heading_range[1] > heading >= heading_range[0]:
                    valid_structure = True
                n_trials_structure += 1
        if n_trials_structure >= max_trials_structure:
            print('get random start and end Failed to find a pair valid for the structure ')
        return start, end

    def get_start_and_end_with_pop_density(self, heading_range=None, protected_area_start=None):
        if self.lz_coordinates is None:
            if self.demand['type'] == 'population_density':
                with open(self.demand['parameters']) as f:
                    data = json.load(f)
                    self.lz_coordinates = np.array(data['coordinates_xy'], dtype=np.float64)
                    self.cumulative_densities = data['cumulative_distribution']
            else:
                print('this demand type is not implemented ' + str(self.demand))
        if heading_range is None:
            valid_structure = True
        else:
            valid_structure = False
        valid_start_and_end = False
        n_trials_structure = 0
        max_trials_structure = 100
        while (not valid_structure or not valid_start_and_end) and n_trials_structure < max_trials_structure:
            # Select start
            val1 = random.random()
            index1 = np.searchsorted(self.cumulative_densities, val1)
            start = self.lz_coordinates[index1]
            if protected_area_start is not None:
                number_of_trials = 0
                max_number_of_trials = 10
                while np.linalg.norm(protected_area_start['center'] - start) <= protected_area_start['radius'] and number_of_trials < max_number_of_trials:
                    number_of_trials += 1
                    val1 = random.random()
                    index1 = np.searchsorted(self.cumulative_densities, val1)
                    start = self.lz_coordinates[index1]
                if np.linalg.norm(protected_area_start['center'] - start) <= protected_area_start['radius']:
                    print(start)
                    print(protected_area_start['center'])
                    self.my_logger.log('getRandomStartAndEnd failed to place the random agent in a conflict free zone')
            # Select goal
            valid_start_and_end = False
            max_number_of_trials = 10
            number_of_trials = 0
            while not valid_start_and_end and number_of_trials < max_number_of_trials:
                number_of_trials += 1
                val2 = random.random()
                index2 = np.searchsorted(self.cumulative_densities, val2)
                goal = self.lz_coordinates[index2]
                if np.linalg.norm(start - goal) > self.tolerance:
                    valid_start_and_end = True
            # Check the structure to ensure that start and end
            if heading_range is not None:
                heading = math.atan2(goal[1] - start[1], goal[0] - start[0]) * 180 / math.pi
                if heading_range[1] > heading >= heading_range[0]:
                    valid_structure = True
                n_trials_structure += 1
        if n_trials_structure >= max_trials_structure:
            print('get_start_and_end_with_demand failed to find a pair valid for the structure')
        return start, goal

    def get_start_and_end_with_hub_and_spoke(self, heading_range=None, protected_area_start=None):
        if self.lz_coordinates is None:
            if self.demand['type'] == 'hub_and_spoke':
                with open(self.demand['parameters']) as f:
                    data = json.load(f)
                    self.lz_coordinates = {}
                    for k in data:
                        self.lz_coordinates[int(k)] = {'distribution_center': np.array(data[k]['distribution_center'], dtype=np.float64),
                                                       'customers': np.array(data[k]['customers'], dtype=np.float64)}
        if heading_range is not None:
            self.my_logger.log('Hub and spoke not really compatible with single layer study')
        valid_start_and_end = False
        max_number_of_trials = 100
        n_trials = 0
        while not valid_start_and_end and n_trials < max_number_of_trials:
            n_trials += 1
            index_start = random.randrange(0, len(self.lz_coordinates))
            start = self.lz_coordinates[index_start]['distribution_center']
            if protected_area_start is not None:
                number_of_trials = 0
                max_number_of_trials = 10
                while np.linalg.norm(protected_area_start['center'] - start) <= protected_area_start['radius'] and number_of_trials < max_number_of_trials:
                    number_of_trials += 1
                    index_start = random.randrange(0, len(self.lz_coordinates))
                    start = self.lz_coordinates[index_start]['distribution_center']
                if np.linalg.norm(protected_area_start['center'] - start) <= protected_area_start['radius']:
                    print(start)
                    print(protected_area_start['center'])
                    self.my_logger.log('getRandomStartAndEnd failed to place the random agent in a conflict free zone')
            index_end = random.randrange(0, len(self.lz_coordinates[index_start]['customers']))
            goal = self.lz_coordinates[index_start]['customers'][index_end]
            if np.linalg.norm(start - goal) > self.tolerance:
                valid_start_and_end = True
        if n_trials >= max_number_of_trials:
            self.my_logger.log('exceeded max number of trials to place random start and end for hub and spoke')
        return start, goal

    def terminate(self):
        # Go through all the conflicts and agents that are leftovers
        combined_log = {'conflicts': [], 'agents': [], 'times': {'time_density_is_reached': self.t_density, 'time_all_started_after_t_density': self.t0}}
        if self.log_density:
            combined_log['density_map'] = []
        for layer in self.layers:
            log_data = layer.terminate()
            combined_log['conflicts'].append(log_data['conflicts'])
            combined_log['agents'].append(log_data['agents'])
            if self.log_density:
                combined_log['density_map'].append(log_data['density_map'])
        return combined_log

    def add_agents(self, time, agent_types=None):
        if self.multiple_planning_agents:
            if agent_types is None:
                # Depending on the phase, add agents to the simulation to increase the total number of agents
                if self.ramping_up:
                    time_interval = 1570.0 / self.desired_number_of_agents
                    if time - self.agent_added_time > time_interval:
                        self.agent_added_time = time
                        a, layer_id = self.create_agent(time)
                        self.layers[layer_id].add_agent(time, a)
                if self.n_priority_agents != 0 and self.ramping_up_priority:
                    time_interval = 1570.0 / self.n_priority_agents
                    if time - self.priority_agent_added_time > time_interval:
                        self.priority_agent_added_time = time
                        a, layer_id = self.create_agent(time, priority=True)
                        self.layers[layer_id].add_agent(time, a)
            else:
                for agent_type in agent_types:
                    if agent_type['type'] == 'dumb':
                        priority = True
                    else:
                        priority = False
                    if agent_type['parameters'] is not None:
                        # Only use case, return to base for hub and spoke demand
                        flight_leg = agent_type['parameters']['flight_leg']
                        start = agent_type['parameters']['start']
                        end = agent_type['parameters']['goal']
                        a, layer_id = self.create_agent(time, priority=priority, flight_leg=flight_leg, my_start=start, my_end=end)
                    else:
                        a, layer_id = self.create_agent(time, priority=priority)
                    self.layers[layer_id].add_agent(time, a)

    def create_agent(self, time, priority=False, flight_leg='initial', my_start=None, my_end=None):
        if priority:
            agent_logic = 'dumb'
            algo_type = None
        else:
            agent_logic = self.simulation_type
            algo_type = self.algo_type

        if self.structure is None:
            # Assign layer randomly
            layer_id = random.randint(0, self.n_layers - 1)
            # Create random start and end based on demand if there is one
            if my_start is None:
                if not priority:
                    start, goal = self.get_random_start_and_end()
                else:
                    start, goal = self.get_random_start_and_end_outside()
            else:
                start = my_start
                goal = my_end
        else:
            # There is a structure
            layer_ranges = np.array(self.structure['parameters'])
            if len(layer_ranges.shape) == 1:
                # There is only one range, create random start and end in this range not compatible with non-uniform demand
                self.my_logger.log('There is only one range in the structure')
                if my_start is not None:
                    self.my_logger.log('structure and demand are incompatible')
                layer_id = 0
                if not priority:
                    start, goal = self.get_random_start_and_end(heading_range=layer_ranges)  # TODO enforce the range
                else:
                    start, goal = self.get_random_start_and_end_outside()
            else:
                # There are multiple layers
                if priority:
                    # ignore rules
                    if my_start is None:
                        start, goal = self.get_random_start_and_end_outside()
                    else:
                        start = my_start
                        goal = my_end
                    layer_id = random.randint(0, self.n_layers - 1)
                else:
                    if my_start is None:
                        # Create a random start and end
                        start, goal = self.get_random_start_and_end()
                    else:
                        start = my_start
                        goal = my_end
                    # Figure out in which layer it belongs
                    heading = math.atan2(goal[1] - start[1], goal[0] - start[0]) * 180.0 / math.pi
                    i = 0
                    for l_range in layer_ranges:
                        if l_range[0] <= heading <= l_range[1]:
                            layer_id = i
                            break
                        i += 1
                    if i >= len(layer_ranges):
                        print('The heading is: ' + str(heading))
                        self.my_logger.log('There is an issue with the structure and the layers, the structure does not cover all potential headings')
                        layer_id = None
        # Create the agent
        a = agent.Agent(self, self.min_distance, self.max_speed, start=start, end=goal, start_time=time, agent_logic=agent_logic, flight_leg=flight_leg,
                        algo_type=algo_type, id=self.total_number_of_agents)
        self.total_number_of_agents += 1
        return a, layer_id

    def get_all_agents(self):
        agents_dic = {}
        for i in range(0, self.n_layers):
            agents_dic[i] = self.layers[i].get_all_agents()
        return agents_dic

    def get_protected_area(self):
        return None
