from queue import Queue
from utm_simulator import simulation
from utm_simulator import display
import random

if __name__ == '__main__':
    # Simulation parameters
    random.seed(1)

    minimum_separation = 500  # m
    length_arena = 20000  # m
    min_speed = 0
    max_speed = 20.0  # m/s
    sensing_radius = 5000
    time_step = 1
    simulation_length = float('inf')
    n_agent_results = 100

    n_intruder = 50  # 300
    n_priority = 0  # 5
    simulation_type = 'reactive'  # 'reactive' or 'strategic'
    algo_type = 'MVP_Bluesky' # reactive: MVP_Bluesky, ORCA, straight; strategic: Decoupled, LocalVO, SIPP
    # structure = {'type': 'layer', 'parameters': [[0, 180], [-180, 0]]}
    structure = None
    # demand = None
    demand = {'type': 'hub_and_spoke', 'parameters': 'data/atlanta_obs_and_hub_200.json'}
    # demand = {'type': 'population_density', 'parameters': 'data/atl_obs_and_pop_density_00.json'}
    static_obstacles = {'type': 'circle', 'parameters': 'data/atlanta_obstacles_200.json'}
    # static_obstacles = None
    save_all = True

    multiple_planning_agents = True

    update_queue = Queue()
    # update_queue = None

    sim = simulation.Simulation(length_arena, n_intruder, minimum_separation, max_speed,
                                time_step=time_step, time_end=simulation_length, multiple_planning_agents=multiple_planning_agents,
                                structure=structure, demand=demand, static_obstacles=static_obstacles, n_priority_agents=n_priority,
                                simulation_type=simulation_type, simulation_name='test', algorithm_type=algo_type, sensing_radius=sensing_radius,
                                log_type='short', save_file=save_all, log_density=False, n_valid_agents_for_simulation_end=n_agent_results,
                                update_queue=update_queue, stdout_to_file=True)
    dis = display.Display(update_queue, length_arena, display_update=20 * time_step, static_obstacles=static_obstacles)
    sim.run()
    dis.run()
