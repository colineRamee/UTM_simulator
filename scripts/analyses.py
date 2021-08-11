import numpy as np
import itertools


def average_throughput(data, t_0_measure=0):
    """ Return throughput in agents per minute exiting the simulation (start counting agents after t_0_measure"""
    i = 0
    for agent in flatten(data["agents"]):
        if agent["flight_status"] == "finished" and agent["actual_time_of_arrival"] >= t_0_measure:
            i += 1
            final_time = agent["time_removed_from_sim"]
    return 60 * i / (final_time - t_0_measure)


def los_severity(data, t_0_measure=0, desired_separation_distance=500):
    conflicts = flatten(data["conflicts"])
    hip = []
    for conflict in merge_los(conflicts):
        if conflict["start_time"] >= t_0_measure:
            hip.append(1 - conflict["min_separation"] / desired_separation_distance)
    if hip == []:
        hip.append(0)
    return hip


def count_los(data, t_0_measure=0):
    conflicts = flatten(data["conflicts"])
    n_los = 0
    for conflict in merge_los(conflicts):
        if conflict["start_time"] >= t_0_measure:
            n_los += 1
    return n_los


def merge_los(los_list):
    unique_conflicts = {}
    for los in los_list:
        agents_set = frozenset([los['agent1'], los['agent2']])
        if agents_set not in unique_conflicts:
            unique_conflicts[agents_set] = los
        else:
            unique_conflicts[agents_set]['min_separation'] = min(unique_conflicts[agents_set]['min_separation'], los["min_separation"])
            unique_conflicts[agents_set]['min_h_separation'] = min(unique_conflicts[agents_set]['min_h_separation'], los["min_h_separation"])
            unique_conflicts[agents_set]['min_z_separation'] = min(unique_conflicts[agents_set]['min_z_separation'], los["min_z_separation"])
            unique_conflicts[agents_set]['start_time'] = min(unique_conflicts[agents_set]['start_time'], los["start_time"])
            unique_conflicts[agents_set]['end_time'] = max(unique_conflicts[agents_set]['end_time'], los["end_time"])
    return unique_conflicts.values()


def los_per_agent(data, t_0_measure=0):
    agents = flatten(data["agents"])
    conflicts = merge_los(flatten(data["conflicts"]))
    agents_los = {}
    los = []
    for conflict in conflicts:
        agents_los[conflict['agent1']] = agents_los.setdefault(conflict['agent1'], 0) + 1
        agents_los[conflict['agent2']] = agents_los.setdefault(conflict['agent2'], 0) + 1
    n_valid_agents = 1000
    n = 0
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            n += 1
            if agent['agent_id'] in agents_los:
                los.append(agents_los[agent['agent_id']])
            else:
                los.append(0)
        if n >= n_valid_agents:
            break
    return los


def ground_delay(data, t_0_measure=0):
    agents = flatten(data["agents"])
    delays = []
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            delays.append(agent["actual_time_of_departure"] - agent["desired_time_of_departure"])
    return np.array(delays)


def time_to_think(data, t_0_measure=0):
    agents = flatten(data["agents"])
    time_to_compute = []
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            if agent['agent_type'] == 'reactive':
                time_to_compute.append(agent["total_planning_time"])
            else:
                time_to_compute.append(agent["time_to_preflight"])
    return np.array(time_to_compute)


def los_per_agent_flight_hour(data, t_0_measure=0):
    agents = flatten(data["agents"])
    conflicts = merge_los(flatten(data["conflicts"]))
    agents_los = {}
    n_los = 0.0
    tot_time = 0.0
    for conflict in conflicts:
        agents_los[conflict['agent1']] = agents_los.setdefault(conflict['agent1'], 0) + 1
        agents_los[conflict['agent2']] = agents_los.setdefault(conflict['agent2'], 0) + 1
    n_valid_agents = 10000
    n = 0
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            n += 1
            if agent['agent_id'] in agents_los:
                n_los += agents_los[agent['agent_id']]
            tot_time += agent["actual_time_of_arrival"] - agent["actual_time_of_departure"]
        if n >= n_valid_agents:
            break
    # tot_time in seconds
    return 3600.0 * n_los / tot_time


def percentage_cancelled_flights(data, t_0_measure=0):
    agents = flatten(data["agents"])
    n_cancelled = 0
    n_finished = 0
    for agent in agents:
        if agent["flight_status"] == 'cancelled' and agent["desired_time_of_departure"] >= t_0_measure:
            n_cancelled += 1
        if agent["flight_status"] == 'finished' and agent["desired_time_of_departure"] >= t_0_measure:
            n_finished += 1
    return n_cancelled / n_finished


def percentage_stuck_flights(data, t_0_measure=0):
    agents = flatten(data["agents"])
    n_stuck = 0
    n_finished = 0
    for agent in agents:
        if agent["flight_status"] == 'stuck' and agent["desired_time_of_departure"] >= t_0_measure:
            n_stuck += 1
        if agent["flight_status"] == 'finished' and agent["desired_time_of_departure"] >= t_0_measure:
            n_finished += 1
    return n_stuck / n_finished


def percentage_perturbed_agents(data, t_0_measure=0):
    agents = flatten(data["agents"])
    n_perturbed = 0
    n_total = 0
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            n_total += 1
            if agent["perturbed_by_priority_agent"]:
                n_perturbed += 1
    return n_perturbed / n_total


def nmac_per_agent_flight_hour(data, t_0_measure=0, nmac=150, variable_separation_distance=False):
    agents = flatten(data["agents"])
    conflicts = merge_los(flatten(data["conflicts"]))
    agents_los = {}
    n_los = 0.0
    tot_time = 0.0
    for conflict in conflicts:
        if variable_separation_distance:
            limit = (nmac / 500.0) * conflict["desired_separation"]
        else:
            limit = nmac
        if conflict["min_separation"] <= limit:
            agents_los[conflict['agent1']] = agents_los.setdefault(conflict['agent1'], 0) + 1
            agents_los[conflict['agent2']] = agents_los.setdefault(conflict['agent2'], 0) + 1
    n_valid_agents = 10000
    n = 0
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            n += 1
            if agent['agent_id'] in agents_los:
                n_los += agents_los[agent['agent_id']]
            tot_time += agent["actual_time_of_arrival"] - agent["actual_time_of_departure"]
        if n >= n_valid_agents:
            break
    # tot_time in seconds
    return 3600.0 * n_los / tot_time


def severity_per_valid_los(data, t_0_measure=0, desired_separation_distance=500, variable_separation_distance=False):
    """A valid los is a los that involves at least one valid agent"""
    agents = flatten(data['agents'])
    valid_agents = []
    n_valid_agents = 1000
    n = 0
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            n += 1
            valid_agents.append(agent['agent_id'])
        if n >= n_valid_agents:
            break
    conflicts = merge_los(flatten(data["conflicts"]))
    severity = []
    for conflict in conflicts:
        if conflict['agent1'] in valid_agents or conflict['agent2'] in valid_agents:
            if variable_separation_distance:
                severity.append(1 - conflict["min_separation"] / conflict["desired_separation"])
            else:
                severity.append(1 - conflict["min_separation"] / desired_separation_distance)
    if severity == []:
        severity.append(0)
    return severity


def efficiency(data, t_0_measure=0):
    agents = flatten(data["agents"])
    times = []
    t_efficiency = []
    e_efficiency = []
    n_valid_agents = 1000
    n = 0
    for agent in agents:
        if agent["flight_status"] == 'finished' and agent["time_removed_from_sim"] >= t_0_measure:
            n += 1
            delta_t_ideal = agent["ideal_time_of_arrival"] - agent["desired_time_of_departure"]
            eta_t = delta_t_ideal / (agent["actual_time_of_arrival"] - agent["desired_time_of_departure"])
            t_efficiency.append(eta_t)
            eta_e = delta_t_ideal / (agent["actual_time_of_arrival"] - agent["actual_time_of_departure"])
            e_efficiency.append(eta_e)
            if 'time_removed_from_sim' in agent:
                times.append(agent["time_removed_from_sim"])
            else:
                times.append(agent["actual_time_of_arrival"])
            if eta_e > 1.1:
                print(agent['agent_id'])
                print(agent)
                print(eta_e)
                print(eta_t)
        if n >= n_valid_agents:
            break
    return np.array(t_efficiency), np.array(e_efficiency), np.array(times)


def flatten(my_list):
    # only flatten if it's a list of list
    if my_list != []:
        if type(my_list[0]) is list:
            # there were multiple layers, let's flatten them
            return list(itertools.chain.from_iterable(my_list))
    return my_list


def coline_boxplot(ax, data, x_axis, index, n_categories, color, label=None):
    width = 0.8 / n_categories
    x_offset = -width * (n_categories - 1) / 2 + index * width
    x = [i + x_offset for i in range(0, len(data))]
    # Empty line (trick to get the legend to work)
    ax.plot([], [], label=label, color=color)
    bplot = ax.boxplot(data, positions=x, widths=width, boxprops=dict(color=color), medianprops=dict(color='black'),
                       whiskerprops=dict(color=color), capprops=dict(color=color), patch_artist=True, whis=[5, 95], flierprops=dict(markersize=1))
    for box in bplot['boxes']:
        box.set_facecolor(color)
    if index == n_categories - 1:
        x_marks = [i for i in range(0, len(x_axis))]
        ax.set_xticks(x_marks)
        ax.set_xticklabels(x_axis)


def fill_dic_table(table_dic, method, index, q=0, eta_t=0, eta_e=0, n_los_h=None, n_nmac_h=None, runtime=None):
    if method == 'straight':
        method_name = 'Baseline'
    elif method == 'MVP_Bluesky':
        method_name = 'MVP'
    else:
        method_name = method
    table_dic[(method_name, 'q')] = q[index]
    table_dic[(method_name, 'eta_t')] = eta_t[index]
    table_dic[(method_name, 'eta_e')] = eta_e[index]
    if n_los_h is not None:
        table_dic[(method_name, 'n_los_h')] = n_los_h[index]
    if n_nmac_h is not None:
        table_dic[(method_name, 'n_nmac_h')] = n_nmac_h[index]
    if runtime is not None:
        table_dic[(method_name, 'runtime')] = runtime[index]


def dic_to_latex(table_dic, txt_file):
    with open(txt_file, 'w') as f:
        f.write('Algorithm & Throughput & $\eta_{time}$ & $\eta_{energy}$ & $n_{LoS}/h$ & $n_{NMAC}/h$ \\\\\\hline \n')
        for method in ['Baseline', 'Decoupled', 'LocalVO', 'SIPP', 'MVP', 'ORCA']:
            f.write(method)
            for metric in ['q', 'eta_t', 'eta_e', 'n_los_h', 'n_nmac_h', 'runtime']:
                if (method, metric) in table_dic:
                    f.write(' & ' + '{:0.2g}'.format(table_dic[(method, metric)]))
            f.write(' \\\\ \n')
