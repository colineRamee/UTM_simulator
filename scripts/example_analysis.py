import os
import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib as mpl
from scripts.analyses import efficiency, severity_per_valid_los, los_per_agent, los_per_agent_flight_hour, nmac_per_agent_flight_hour, average_throughput, ground_delay, time_to_think, coline_boxplot
import pickle

# Load results
# This can take a minute so the results are pickled, and this section can be commented when working on the plots

results = {}
my_path = '../logs/example/'
for filename in os.listdir(my_path):
    with open(my_path + filename) as f:
        data = json.load(f)
    t0 = data["times"]["time_all_started_after_t_density"]
    algo_type = data["inputs"]["algorithm_type"]
    n_intruders = data["inputs"]["n_intruders"]
    sim_type = data["inputs"]["simulation_type"]
    if algo_type == 'straight':
        sim_type = 'baseline'
    experiment = results.setdefault(sim_type, {}).setdefault(algo_type, {}).setdefault(n_intruders, {})
    t_efficiency, e_efficiency, t = efficiency(data, t_0_measure=t0)
    experiment.setdefault('time_efficiency', []).append(t_efficiency)
    experiment.setdefault('energy_efficiency', []).append(e_efficiency)
    hip = severity_per_valid_los(data, t_0_measure=t0, desired_separation_distance=data["inputs"]["h_collision_dist_m"])
    experiment.setdefault('hip', []).append(hip)
    n_los = los_per_agent(data, t_0_measure=t0)
    experiment.setdefault('los_per_agent', []).append(n_los)
    n_los_per_flight_hour = los_per_agent_flight_hour(data, t_0_measure=t0)
    experiment.setdefault('los_per_agent_flight_hour', []).append(n_los_per_flight_hour)
    n_nmac_per_agent_flight_hour = nmac_per_agent_flight_hour(data, t_0_measure=t0, nmac=152.4)
    experiment.setdefault('nmac_per_agent_flight_hour', []).append(n_nmac_per_agent_flight_hour)
    q = average_throughput(data, t_0_measure=t0)
    experiment.setdefault('throughput', []).append(q)
    ground_delays = ground_delay(data, t_0_measure=t0)
    experiment.setdefault('ground_delays', []).append(ground_delays)
    compute_time = time_to_think(data, t_0_measure=t0)
    experiment.setdefault('compute_time', []).append(compute_time)

pickle.dump(results, open("saved_results.p", "wb"))

# You only need to run the above section once

results = pickle.load(open("saved_results.p", "rb"))

# Create plots
mpl.rcParams['font.size'] = 12
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = 'Times New Roman'

# Throughput
fig0, ax0 = plt.subplots(1, 1, figsize=(6, 4))
plt.tight_layout()
fig0.subplots_adjust(top=0.75)

# Efficiencies
fig1, ax1 = plt.subplots(1, 2, figsize=(6, 4))
fig1.subplots_adjust(top=0.75, wspace=0.3)

# Throughput distribution across 10 runs
fig_q_distrib, ax_q_distrib = plt.subplots(1, 1, figsize=(6, 4))
ax_q_distrib.set_ylabel('Q (agents/min)')
ax_q_distrib.set_xlabel('N')

# Efficiencies
fig2, ax2 = plt.subplots(2, 1, figsize=(6, 5))
fig2.subplots_adjust(top=0.83, hspace=0.05, right=0.99, left=0.11)

# Safety (LoS/h and average HIP)
fig3, ax3 = plt.subplots(2, 1, figsize=(6, 4))
fig3.subplots_adjust(top=0.85, hspace=0.2)

# Safety (HIP distribution and NMAC/h)
fig4, ax4 = plt.subplots(2, 1, figsize=(6, 4))
fig4.subplots_adjust(top=0.85, hspace=0.2)

# Runtime
fig5, ax5 = plt.subplots(1, 1, figsize=(6, 3))
plt.tight_layout()

offset = 0
i = -1
j = -1
n_categories = 6
for simulation_type in results:
    if simulation_type == 'reactive':
        access = 'Free-access, '
    elif simulation_type == 'strategic':
        access = '4DT contract, '
    else:
        access = ''
    for algo_type in results[simulation_type]:
        if algo_type == 'MVP_Bluesky':
            label = access + 'MVP'
        elif algo_type == 'LocalVO':
            label = access + 'Local VO'
        elif algo_type == 'straight':
            label = 'Baseline, straight'
        else:
            label = access + algo_type

        i += 1
        n_intruders = []
        q = []
        eta_e = []
        eta_t = []
        los = []
        hip = []
        los_per_hour = []
        nmac_per_hour = []
        throughput = []
        energy_efficiency = []
        time_efficiency = []
        loss_of_separation = []
        horizontal_intrusion_parameter = []
        compute_time = []
        for n in sorted(results[simulation_type][algo_type]):
            runs = results[simulation_type][algo_type][n]
            n_intruders.append(n)
            q.append(sum(runs['throughput']) / len(runs['throughput']))
            los_per_hour.append(sum(runs['los_per_agent_flight_hour']) / len(runs['los_per_agent_flight_hour']))
            nmac_per_hour.append(sum(runs['nmac_per_agent_flight_hour']) / len(runs['nmac_per_agent_flight_hour']))
            throughput.append(runs['throughput'])
            eff = list(np.concatenate(runs['energy_efficiency']))
            eta_e.append(np.mean(eff))
            energy_efficiency.append(eff)
            eff = list(np.concatenate(runs['time_efficiency']))
            eta_t.append(np.mean(eff))
            time_efficiency.append(eff)
            lossOfSeparation = list(np.concatenate(runs['los_per_agent']))
            los.append(np.mean(lossOfSeparation))
            loss_of_separation.append(lossOfSeparation)
            intrusion = list(np.concatenate(runs['hip']))
            hip.append(np.mean(intrusion))
            horizontal_intrusion_parameter.append(intrusion)
            think = list(np.concatenate(runs['compute_time']))
            compute_time.append(np.mean(think))

        p = ax0.plot(n_intruders, q, label=label)
        color = p[-1].get_color()
        ax1[0].plot(n_intruders, eta_e, label=label)
        ax1[1].plot(n_intruders, eta_t, label=label)
        # ax1bis[0].plot(n_intruders, los, label=label)
        # ax1bis[1].plot(n_intruders, hip, label=label)
        if algo_type != 'straight':
            ax5.plot(n_intruders, compute_time, label=label)
        if simulation_type != 'strategic':
            j += 1
            if algo_type == 'ORCA':
                ax3[0].plot(n_intruders[1:], los_per_hour[1:], label=label, color=color)
            else:
                ax3[0].plot(n_intruders, los_per_hour, label=label, color=color)
            ax3[1].plot(n_intruders, hip, label=label, color=color)
            ax4[1].plot(n_intruders, nmac_per_hour, label=label, color=color, marker='o', linestyle='None', fillstyle='none', mew=2)
            coline_boxplot(ax4[0], horizontal_intrusion_parameter, n_intruders, j, 3, color, label=label)

        coline_boxplot(ax_q_distrib, throughput, n_intruders, i, n_categories, color, label=label)
        if algo_type != 'straight':
            coline_boxplot(ax2[0], energy_efficiency, n_intruders, i + offset, n_categories - 1, color, label=label)
            coline_boxplot(ax2[1], time_efficiency, n_intruders, i + offset, n_categories - 1, color, label=label)
        else:
            offset = -1

# Throughput
ax0.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, borderaxespad=0.)
ax0.set_ylim(bottom=0)
ax0.set_xlim(left=0)
ax0.set_ylabel('Q (agents/min)')
ax0.set_xlabel('N')
ax0.grid()

ax1[0].legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, borderaxespad=0.)
ymin, ymax = ax1[1].get_ylim()
ax1[0].set_ylim(ymin, ymax)
ax1[0].set_ylabel(r"$\eta_{energy}$")
ax1[0].set_xlabel('N')
ax1[0].set_xlim(left=0)
ax1[0].grid()
ax1[1].set_ylabel(r"$\eta_{time}$")
ax1[1].set_xlabel('N')
ax1[1].set_xlim(left=0)
ax1[1].grid()

ax2[0].legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, borderaxespad=0.)
ax2[0].set_ylabel(r"$\eta_{energy}$")
ax2[0].set_ylim(bottom=0, top=1.1)
ax2[0].grid()
ax2[1].set_ylabel(r"$\eta_{time}$")
ax2[1].set_xlabel('N')
ax2[1].set_ylim(bottom=0, top=1.1)
ax2[1].grid()

ax3[0].legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, borderaxespad=0.)
ax3[0].grid()
ax3[0].set_ylabel(r"$n_{LOS}/h$")
ax3[0].set_yscale('log')
ax3[0].set_yticks([0.1, 0.3, 1, 3, 10, 30, 100])
ax3[0].get_yaxis().set_major_formatter(mpl.ticker.ScalarFormatter())
ax3[1].set_ylabel('HIP')
ax3[1].set_xlabel('N')
ax3[1].grid()

ax4[0].legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, borderaxespad=0.)
ax4[0].grid()
ax4[1].set_ylabel(r"$n_{NMAC}/h$")
ax4[0].set_ylabel('HIP')
ax4[1].set_xlabel('N')
ax4[1].grid()
ax4[1].set_yscale('log')
ax4[1].set_yticks([0.001, 0.01, 0.1, 1, 10, 100])

ax5.set_ylim(bottom=0)
ax5.set_xlim(left=0)
ax5.set_xlabel('N')
ax5.set_ylabel('Planning time (s)')
ax5.grid()
ax5.legend()
plt.tight_layout()

plt.show()
