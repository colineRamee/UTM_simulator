import numpy as np
import math
import json

from sklearn.neighbors import KDTree

from utm_simulator.my_utils import MyLogger
from utm_simulator.centralized_manager import SIPPCentralizedManager, VOCentralizedManager, CentralizedManager


class Layer:
    def __init__(self, size_map, min_distance, agent_speed, sensing_radius, tolerance, utm_on, static_obstacles=None, demand=None, default_planning_time_step=None,
                 multiple_planning_agents=False, simulation_type=None, algo_type=None, log_type='short', log_density=False, logger=MyLogger()):
        self.agent_pos = []
        self.agent_record_list = []
        self.conflicts_list = []
        self.conflicts = set()
        self.conflicts_dic = {}
        self.size_map = size_map
        self.min_distance = min_distance
        self.speed = agent_speed
        self.sensing_radius = sensing_radius
        self.tolerance = tolerance
        self.utm_on = utm_on
        self.simulation_type = simulation_type
        self.demand = demand
        self.static_obstacles = static_obstacles
        self.static_obstacles_list = []
        self.default_planning_time_step = default_planning_time_step
        self.multiple_planning_agents = multiple_planning_agents
        self.algo_type = algo_type
        self.log_type = log_type
        self.log_density = log_density
        self.my_logger = logger
        self.smart_agents = []  # Agents that are being studied
        self.dumb_agents = []  # Agents that are not reacting to other agents
        self.active_agents = []  # Smart agents and dumb agents that are in the air
        self.active_agent_pos = []
        self.phantom_agents = []
        self.waiting_agent = []
        self.centralized_manager = None
        self.agents_kd_tree = None
        if self.utm_on:
            if self.algo_type == 'SIPP':
                self.centralized_manager = SIPPCentralizedManager(self.min_distance, self.size_map)
            elif self.algo_type == 'LocalVO':
                self.centralized_manager = VOCentralizedManager(self.min_distance, size_area=self.size_map)
            elif self.algo_type == 'Decoupled':
                self.centralized_manager = CentralizedManager(self.min_distance, size_area=self.size_map)
            else:
                self.my_logger.log('There is no centralized manager defined for this algorithm type')

        if self.static_obstacles is not None:
            self.load_static_obstacles()
            if self.centralized_manager is not None:
                self.centralized_manager.add_static_obstacles(self.static_obstacles_list)

    def run(self, time, sim_dt):
        self.time = time
        self.dt = sim_dt
        waiting_agents_ready_to_take_off = []
        for a in self.waiting_agent:
            if a.can_safely_take_off(self.time, waiting_agents_ready_to_take_off):
                waiting_agents_ready_to_take_off.append(a)
        for a in waiting_agents_ready_to_take_off:
            # Assume only smart agents can wait on the ground
            self.waiting_agent.remove(a)
            self.smart_agents.append(a)
        density = len(self.smart_agents) + len(self.dumb_agents)
        for a in self.smart_agents + self.dumb_agents:
            if self.time >= a.start_time:
                a.compute_next_move(self.time, sim_dt, density=density)
        for a in self.phantom_agents:
            a.compute_next_move(self.time, sim_dt)
        self.active_agents = []
        self.active_agent_pos = []
        for a in self.smart_agents + self.dumb_agents:
            if self.time >= a.start_time:
                self.active_agents.append(a)
                pos = a.move()
                self.active_agent_pos.append(pos)
        for a in self.phantom_agents:
            a.move()
        return len(self.waiting_agent) + len(self.smart_agents)

    def remove_outsiders(self, time, t_density, t0):
        agents_to_remove = []
        found_old_agents = False
        agent_oldest_time = time
        valid_density_finished_agents = 0
        for a in self.smart_agents + self.dumb_agents:
            dist_to_goal = np.linalg.norm(a.position - a.goal)
            if a.start_time < agent_oldest_time:
                agent_oldest_time = a.start_time
            if a.status == 'stuck':
                self.agent_record_list.append(a.log_agent())
                agents_to_remove.append(a)
            elif dist_to_goal <= self.tolerance:
                a.finish_flight(time + dist_to_goal / a.maxSpeed, goal_pos=a.goal, t_removed_from_sim=time)
                agents_to_remove.append(a)
                if self.log_type != 'short' or a.agent_logic != 'dumb':
                    self.agent_record_list.append(a.log_agent())
                if t0 is not None and a.agent_logic != 'dumb' and a.desired_start_time >= t0:
                    valid_density_finished_agents += 1
            if t_density is not None and t0 is None:
                if a.start_time < t_density:
                    found_old_agents = True
        agents_to_compensate = []
        for a in agents_to_remove:
            new_agent_info = self.remove_agent(a, time)
            if new_agent_info is not None:
                agents_to_compensate.append(new_agent_info)
        return found_old_agents, valid_density_finished_agents, agents_to_compensate

    def add_agent(self, time, agent):
        agent.link_layer(self)
        if agent.agent_logic == 'strategic':
            agent.link_centralized_manager(self.centralized_manager)
            flight_plan = agent.preflight(self.default_planning_time_step, algo_type=self.algo_type)
            if flight_plan is None:
                self.my_logger.log('Strategic agent failed to plan')
                self.agent_record_list.append(agent.log_agent())
            else:
                self.centralized_manager.add_flight_plan(flight_plan, agent.id, agent, time, priority=False)
                self.smart_agents.append(agent)
        elif agent.agent_logic == 'reactive':
            self.waiting_agent.append(agent)
        elif agent.agent_logic == 'dumb':
            if self.utm_on:
                agent.link_centralized_manager(self.centralized_manager)
                density = 0
                flight_plan = agent.preflight(self.default_planning_time_step, density=density)
                self.centralized_manager.add_flight_plan(flight_plan, agent.id, agent, time, priority=True)
            self.dumb_agents.append(agent)
        else:
            print('Layer, add agent error, unrecognized agent logic type')

    def load_static_obstacles(self):
        if self.static_obstacles['type'] == 'circle':
            with open(self.static_obstacles['parameters']) as f:
                data = json.load(f)
                self.static_obstacles_list = []
                i = 0
                for obstacle in data:
                    i += 1
                    self.static_obstacles_list.append(Obstacle(np.array(obstacle['position']), obstacle['radius'], i))

    def check_collisions(self, time):
        # Naive implementation
        # The measure area should be inside the experimentation area with some margin to avoid the case where an intruder is created right next to the ownship
        current_conflicts = set()
        for a in self.active_agents:
            a.status = 'ok'
        for i in range(0, len(self.active_agents)):
            agentA = self.active_agents[i]
            if agentA.agent_logic != 'dumb':
                # Multiplying the distance by a factor slightly smaller than one to avoid numerical error
                agents = self.get_neighbors(agentA.position, self.min_distance * 0.99999)
                for agentB in agents:
                    # Do not count conflicts between two dumb agents
                    if agentB.id > agentA.id or agentB.agent_logic == 'dumb':
                        current_conflicts.add(frozenset([agentA, agentB]))  # Guarantees that each conflict is counted once
                        agentA.status = 'boom'
                        agentB.status = 'boom'
                for obstacle in self.static_obstacles_list:
                    if np.linalg.norm(obstacle.position - agentA.position) < obstacle.radius:
                        current_conflicts.add(frozenset([agentA, obstacle]))
                        agentA.status = 'boom'
        # Conflicts that are over
        finished_conflicts = self.conflicts - current_conflicts
        for c in finished_conflicts:
            conflict_object = self.conflicts_dic.pop(c)
            self.conflicts_list.append(conflict_object.bundle_conflict())
        continued_conflicts = self.conflicts & current_conflicts
        for c in continued_conflicts:
            conflict_object = self.conflicts_dic[c]
            conflict_object.update(time)
        started_conflicts = current_conflicts - self.conflicts
        for c in started_conflicts:
            a1, a2 = c
            # If the conflict is between an agent and a static obstacle set the minimum distance to be the static obstacle radius
            # Otherwise the desired minimum distance is the same for all agents
            if isinstance(a1.id, str):
                desired_separation = a1.radius
            elif isinstance(a2.id, str):
                desired_separation = a2.radius
            else:
                desired_separation = self.min_distance
            conflict_object = Conflict(time, c, desired_separation)
            self.conflicts_dic[c] = conflict_object
            conflict_object.update(time)
        self.conflicts = current_conflicts

    def terminate(self):
        # Go through all the conflicts and agents that are leftovers
        for agent in self.smart_agents + self.dumb_agents:
            if self.log_type != 'short' or agent.agent_logic != 'dumb':
                self.agent_record_list.append(agent.log_agent())
        for agent in self.waiting_agent:
            # The simulation timed out without being able to take-off
            if self.log_type != 'short' or agent.agent_logic != 'dumb':
                self.agent_record_list.append(agent.log_agent())
        for conflict in self.conflicts_dic.values():
            self.conflicts_list.append(conflict.bundle_conflict())
        if self.agent_record_list == []:
            print('The agent log failed to be added')
            print('The waiting list is ', self.waiting_agent)
        log_data = {'conflicts': self.conflicts_list,
                    'agents': self.agent_record_list,
                    # 'times': {'time_density_is_reached': self.t_density, 'time_all_started_after_t_density': self.t0}
                    }
        if self.log_density:
            log_data['density_map'] = self.density_map.density.tolist()
        return log_data

    def remove_agent(self, a, time, density=0):
        if a.agent_logic != "dumb":
            self.smart_agents.remove(a)
        else:
            self.dumb_agents.remove(a)
        if self.utm_on:
            self.centralized_manager.terminate_flight_plan(a.id)
        if a not in self.active_agents:
            print('agents probably started too close to its destination')
            print('agent start ' + str(a.start) + ', and end ' + str(a.goal))
            print('agent type is ' + a.agent_logic)
        i = self.active_agents.index(a)
        del self.active_agents[i]
        del self.active_agent_pos[i]
        if self.multiple_planning_agents:
            if self.demand is not None and self.demand['type'] == 'hub_and_spoke' and a.flight_leg == 'initial':
                start = a.goal
                goal = a.start
                return {'type': a.agent_logic, 'parameters': {'flight_leg': 'return_to_base', 'start': start, 'goal': goal}}
            else:
                if a.agent_logic == 'dumb':
                    return {'type': a.agent_logic, 'parameters': None}
                elif self.simulation_type == 'reactive':
                    return {'type': a.agent_logic, 'parameters': None}

                elif self.simulation_type == 'strategic':
                    return {'type': a.agent_logic, 'parameters': None}
        else:
            if self.simulation_type == 'reactive':
                return {'type': a.agent_logic, 'parameters': None}
        return None

    def build_kd_tree(self):
        if self.active_agent_pos == []:
            self.agents_kd_tree = None
        else:
            self.agents_kd_tree = KDTree(np.array(self.active_agent_pos))

    def get_neighbors(self, position, radius):
        # Will also return itself unless an exclusion id is specified
        if self.agents_kd_tree is not None:
            ind = self.agents_kd_tree.query_radius(position.reshape(1, -1), radius, return_distance=False)
            return np.array(self.active_agents)[ind[0]]
        else:
            return []

    def get_static_obstacles(self, position, radius, sorted=False):
        obstacles = []
        distance_to_obstacle_boundary = []
        for obstacle in self.static_obstacles_list:
            dist = np.linalg.norm(position - obstacle.position)
            if dist < radius + obstacle.radius:
                if sorted:
                    d = dist - obstacle.radius
                    i = np.searchsorted(distance_to_obstacle_boundary, d)
                    distance_to_obstacle_boundary.insert(i, d)
                    obstacles.insert(i, obstacle)
                else:
                    obstacles.append(obstacle)
        return obstacles

    def get_nearest_neighbors(self, position, k, radius):
        """Returns the k-closest neighbors within radius distance"""
        if self.agents_kd_tree is not None:
            k = min(k, len(self.active_agent_pos))
            distances, indices = self.agents_kd_tree.query(position.reshape(1, -1), k=k, return_distance=True)
            max_index = np.searchsorted(distances[0], radius, side='right')
            if max_index == 0:
                return []
            else:
                return np.array(self.active_agents)[indices[0][0:max_index]]
        else:
            return []

    def get_all_agents(self):
        agents = {}
        for agent in self.smart_agents + self.dumb_agents:
            if self.time >= agent.start_time:
                agents[agent.id] = {'x': agent.position[0],
                                    'y': agent.position[1],
                                    'radius': agent.radius,
                                    'status': agent.status,
                                    'ownship': agent.ownship,
                                    'switch_to_reactive': agent.switch_to_reactive}
        return agents


class Conflict:
    def __init__(self, time, agent_set, desired_separation):
        """ The agent_set is a frozenset"""
        # self.agents=agent_set  # Hashable (immutable)
        self.agent1, self.agent2 = agent_set
        self.start_time = time
        self.end_time = time
        self.min_separation = None
        self.min_h_separation = None
        self.min_z_separation = None
        self.desired_separation = desired_separation

    def update(self, time):
        distance = np.linalg.norm(self.agent1.position - self.agent2.position)
        h_distance = distance
        z_distance = 0
        if self.min_separation is None or self.min_separation > distance:
            self.min_separation = distance
        if self.min_h_separation is None or self.min_h_separation > h_distance:
            self.min_h_separation = h_distance
        if self.min_z_separation is None or self.min_z_separation > z_distance:
            self.min_z_separation = z_distance
        self.end_time = time

    def bundle_conflict(self):
        """Returns the conflict attributes in a dictionary"""
        # vars(self) does not work because of the objects
        dic = {'agent1': self.agent1.id, 'agent2': self.agent2.id, 'start_time': self.start_time,
               'end_time': self.end_time,
               'min_separation': self.min_separation, 'min_h_separation': self.min_h_separation,
               'min_z_separation': self.min_z_separation,
               'desired_separation': self.desired_separation}
        return dic


class Obstacle:
    def __init__(self, position, radius, id):
        self.position = position
        self.radius = radius
        self.id = 'fixed_obstacle_' + str(id)


class DensityMap:
    def __init__(self, size_map, size_bin):
        self.size_map = size_map
        self.n_bins = math.ceil(self.size_map / size_bin)
        self.size_bin = self.size_map / self.n_bins
        self.density = np.zeros((self.n_bins, self.n_bins))

    def add_pos(self, position):
        i = min(int(position[0] / self.size_bin),
                self.n_bins - 1)  # truncate (include points on the edge in the last bin)
        j = min(int(position[1] / self.size_bin), self.n_bins - 1)
        self.density[i, j] += 1
