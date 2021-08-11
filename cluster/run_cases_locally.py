import json
import sys
import random

import utm_simulator.simulation as simulation

""" Does the same job as the pbs file for stuff that you don't want to run on the cluster"""
cases = [1]
filename = 'batch_input_use_cases.json'

with open(filename) as f:
    data = json.load(f)

for case in cases:
    if str(case) in data:
        input_parameters = data[str(case)]
        random.seed(input_parameters['random_seed'])
        if 'sensing_radius' in input_parameters:
            # optional parameter
            sensing_radius = input_parameters['sensing_radius']
        else:
            sensing_radius = None
        if 'structure' in input_parameters:
            structure = input_parameters['structure']
        else:
            structure = None
        if 'demand' in input_parameters:
            demand = input_parameters['demand']
        else:
            demand = None
        if 'static_obstacles' in input_parameters:
            static_obstacles = input_parameters['static_obstacles']
        else:
            static_obstacles = None
        if 'n_priority_agents' in input_parameters:
            n_priority_agents = input_parameters['n_priority_agents']
        else:
            n_priority_agents = 0
        if 'n_layers' in input_parameters:
            n_layers = input_parameters['n_layers']
        else:
            n_layers = 1
        sim = simulation.Simulation(input_parameters['length_arena'],
                                    input_parameters['n_intruders'],
                                    input_parameters['minimum_separation'],
                                    input_parameters['max_speed'],
                                    sensing_radius=sensing_radius,
                                    time_step=input_parameters['time_step'],
                                    time_end=input_parameters['time_end'],
                                    simulation_type=input_parameters['simulation_type'],
                                    simulation_name=input_parameters['simulation_name'],
                                    algorithm_type=input_parameters['algorithm_type'],
                                    structure=structure,
                                    demand=demand,
                                    static_obstacles=static_obstacles,
                                    n_priority_agents=n_priority_agents,
                                    multiple_planning_agents=True,
                                    n_layers=n_layers,
                                    log_type='short',
                                    n_valid_agents_for_simulation_end=input_parameters['n_agents_results'],
                                    save_file=False,
                                    stdout_to_file=True)
        log = sim.run()
        log["inputs"]["random_seed"] = input_parameters['random_seed']
        # Save file in specific folder
        filename = 'results/' + input_parameters['simulation_name'] + '.json'
        with open(filename, 'w') as file:
            json.dump(log, file, indent=4)
    else:
        print('The requested simulation number is not in the input file: ' + str(sys.argv[-1]))
