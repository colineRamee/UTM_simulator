import json

""" This file can be used to create the json input file used to run batches of simulations """

# Fixed parameters
minimum_separation = 500  # m
length_arena = 20000  # m
min_speed = 0
max_speed = 20.0  # m/s
simulation_length = float('inf')
n_agents_results = 100

# Number of repetitions of a specific run
n_case = 10

# Variable parameters
n_intruders = [10, 50, 100, 150, 200, 250, 300]
# n_intruders = [300]
algo_types = {#'ORCA': {'simulation_type': 'reactive', 'time_step': 1},
              #'MVP_Bluesky': {'simulation_type': 'reactive', 'time_step': 1},
              'LocalVO': {'simulation_type': 'strategic', 'time_step': 1},
              #'straight': {'simulation_type': 'reactive', 'time_step': 1},
              'SIPP': {'simulation_type': 'strategic', 'time_step': 1}
              #'Decoupled': {'simulation_type': 'strategic', 'time_step': 1}
              }

#structures = [{'type': 'layer', 'parameters': [0, 180]}, {'type': 'layer', 'parameters': [0, 120]}, {'type': 'layer', 'parameters': [0, 90]}]
#structures = [None, {'type': 'layer', 'parameters': [[0, 180], [-180, 0]]}]
structure = None
n_layers = 1
n_priority_agents = [1, 5]
#demand = {'type': 'population_density', 'parameters': 'data/atlanta_lz_pop_density.json'}
# demands = [{'type': 'population_density', 'parameters': '../data/atlanta_lz_pop_density.json'},
#            {'type': 'hub_and_spoke', 'parameters': '../data/atlanta_uav_network.json'}]

# cases = [
#     {'name': 'air_taxi',
#      'demand': {'type': 'population_density', 'parameters': '../data/atl_obs_and_pop_density_800.json'},
#      'static_obstacles': {'type': 'circle', 'parameters': '../data/atlanta_obstacles_800.json'},
#      'n_priority_traffic': 2
#      },
#     {'name': 'drone_delivery',
#      'demand': {'type': 'hub_and_spoke', 'parameters': '../data/atlanta_obs_and_hub_200.json'},
#      'static_obstacles': {'type': 'circle', 'parameters': '../data/atlanta_obstacles_200.json'},
#      'n_priority_traffic': 0
#      }
# ]

# cases = [
#     {'name': 'altitude_200_with_obs',
#      'demand': {'type': 'population_density', 'parameters': '../data/atlanta_lz_density_obstacles200.json'},
#      'static_obstacles': {'type': 'circle', 'parameters': '../data/atlanta_obstacles_200.json'}},
#     {'name': 'altitude_200_no_obs',
#      'demand': {'type': 'population_density', 'parameters': '../data/atlanta_lz_density_obstacles200.json'},
#      'static_obstacles': None},
#     {'name': 'altitude_800_with_obs',
#      'demand': {'type': 'population_density', 'parameters': '../data/atlanta_lz_density_obstacles800.json'},
#      'static_obstacles': {'type': 'circle', 'parameters': '../data/atlanta_obstacles_800.json'}},
#     {'name': 'altitude_800_no_obs',
#      'demand': {'type': 'population_density', 'parameters': '../data/atlanta_lz_density_obstacles800.json'},
#      'static_obstacles': None},
#     {'name': 'uniform_no_obs',
#      'demand': {'type': 'population_density', 'parameters': '../data/atlanta_lz_uniform_density.json'},
#      'static_obstacles': None}
# ]
demand= None
static_obstacles = None
count = 0

input_parameters = {}
for n_priority in n_priority_agents:
#for structure in structures:
    # for case in cases:
    #     demand = case['demand']
    #     static_obstacles = case['static_obstacles']
    #     n_priority = case['n_priority_traffic']
        #n_priority = 0
    for algo_type in algo_types:
        random_seed = 6
        j = 0
        for n in n_intruders:
            j += 1
            for i in range(0, n_case):
                if i == 0:
                    random_seed = j
                else:
                    random_seed = 6 + i + (j-1)*9
                #random_seed += 1
                # if structure is not None:
                #     structure_name = structure['type']+str(structure['parameters'][1])
                #     sim_name=algo_type+'_'+str(n)+'_'+structure_name+'_'+str(random_seed)
                # else:
                #     sim_name=algo_type+'_'+str(n)+'_'+str(random_seed)
                count += 1
                # if structure is None:
                #     sim_name = algo_type + '_' + str(n) + '_' + case['name'] + '_free_' + str(random_seed)
                # else:
                #     sim_name = algo_type+'_'+str(n)+'_'+case['name']+'_'+str(random_seed)
                sim_name = algo_type+'_'+str(n)+'_'+str(n_priority)+'priority_'+str(random_seed)
                run_input = {'length_arena': length_arena, 'n_intruders': n, 'minimum_separation': minimum_separation, 'max_speed': max_speed,
                             'time_step': algo_types[algo_type]['time_step'], 'time_end': simulation_length, 'simulation_type': algo_types[algo_type]['simulation_type'], 'structure': structure,
                             'simulation_name': sim_name, 'algorithm_type': algo_type, 'demand': demand, 'static_obstacles': static_obstacles, 'n_priority_agents': n_priority,
                             # 'simulation_name': algo_type + '_' + str(n) + '_' + case['name'] + '_' + str(random_seed), 'algorithm_type': algo_type, 'demand': case['demand'],
                             # 'static_obstacles': case['static_obstacles'],
                             #'n_layers': n_layers,
                             'n_agents_results': n_agents_results, 'random_seed': random_seed}
                input_parameters[count] = run_input

# Save input file as json
with open('batch_input_priority.json', 'w') as file:
    json.dump(input_parameters, file, indent=4)
