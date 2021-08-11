import numpy as np
import scipy.optimize
import scipy.stats


def build_utm_matrix(results, case, n, additional_param=None):
    j = -1
    matrix = np.zeros((5, 5))
    matrix_n = np.zeros((5, 5))
    alternatives = []
    for simulation_type in results:
        for algo_type in results[simulation_type]:
            if algo_type != 'straight':
                j += 1
                alternatives.append(algo_type)
                if additional_param is not None:
                    runs = results[simulation_type][algo_type][case][additional_param][n]
                else:
                    runs = results[simulation_type][algo_type][case][n]
                q = sum(runs['throughput']) / len(runs['throughput'])
                matrix[0, j] = q
                eta_time = np.mean(list(np.concatenate(runs['time_efficiency'])))
                matrix[1, j] = eta_time
                eta_energy = np.mean(list(np.concatenate(runs['energy_efficiency'])))
                matrix[2, j] = eta_energy
                los_per_hour = sum(runs['los_per_agent_flight_hour']) / len(runs['los_per_agent_flight_hour'])
                matrix[3, j] = los_per_hour
                nmac_per_hour = sum(runs['nmac_per_agent_flight_hour']) / len(runs['nmac_per_agent_flight_hour'])
                matrix[4, j] = nmac_per_hour
    # Normalize the matrix
    print(alternatives)
    for i in range(0, 5):
        matrix_n[i, :] = matrix[i, :]/max(matrix[i, :])
    return matrix_n, matrix


def change_in_ranking_possible(matrix1, matrix2, objective, epsilon=0):
    # Matrices should be normalized and m by n
    # One row per metric (m)
    # One column per alternative (n)
    # Objective, size m, 1 if maximize, -1 if minimize
    n = matrix1.shape[1]  # Number of alternatives
    m = matrix1.shape[0]  # Number of metrics
    # Sum of absolute weight should be 1 (some weights are negative) A_eq @ x = b_eq
    # Objective to minimize does not really matter min c.T @ x
    c = np.zeros((m, 1))
    A_eq = np.zeros((1, m))
    b_eq = 1
    bounds = []
    for i in range(0, m):
        A_eq[0, i] = objective[i]
        bounds.append((min(0, objective[i]), max(0, objective[i])))
        # c[i, 0] = min(0, objective[i])
    b_ub = np.array([-epsilon, -epsilon])
    for i in range(0, n):
        alt1_a = matrix1[:, i]
        alt2_a = matrix2[:, i]
        for j in range(0, n):
            if j != i:
                alt1_b = matrix1[:, j]
                alt2_b = matrix2[:, j]
                delta_1 = alt1_b-alt1_a
                delta_2 = alt2_a-alt2_b
                # Setup the inequalities
                A_ub = np.zeros((2, m))
                A_ub[0, :] = delta_1
                A_ub[1, :] = delta_2
                # Objective maximize the difference in score between the two alternatives
                c = delta_1 + delta_2
                res = scipy.optimize.linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds)
                if res['success']:
                    print('There is a set of weight such that the ranking of alternatives changes between the two matrices')
                    print(res['status'])
                    print('Objective '+str(res['fun']))
                    print('Alternative '+str(i) + ' and '+str(j))
                    x = res['x']
                    print('Weights '+str(x))
                    print('Sum of weights '+str(A_eq @ x))
                    s1_a = alt1_a @ x
                    s1_b = alt1_b @ x
                    print('Matrix 1, Scores: ' + str(s1_a)+', '+str(s1_b))
                    s2_a = alt2_a @ x
                    s2_b = alt2_b @ x
                    print('Matrix 2, Scores: ' + str(s2_a) + ', ' + str(s2_b))
                    #return True
    return False


def decision_to_latex(matrix,matrix_n, weights, file):
    with open(file, 'w') as f:
        f.write('$Q$ & {: 0.2g}& {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.3g} & Maximize \\\\ \n'.format(matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[0, 3], matrix[0, 4], weights[0]))
        f.write('$\\eta_time$ & {: 0.2g}& {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.3g} & Maximize\\\\ \n'.format(matrix[1, 0], matrix[1, 1], matrix[1, 2], matrix[1, 3], matrix[1, 4], weights[1]))
        f.write('$\\eta_energy$ & {: 0.2g}& {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.3g} & Maximize\\\\ \n'.format(matrix[2, 0], matrix[2, 1], matrix[2, 2], matrix[2, 3], matrix[2, 4], weights[2]))
        f.write('$n_los/h$ & {: 0.2g}& {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.3g} & Minimize \\\\ \n'.format(matrix[3, 0], matrix[3, 1], matrix[3, 2], matrix[3, 3], matrix[3, 4], weights[3]))
        f.write('$n_nmac/h$ & {: 0.2g}& {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.2g} & {: 0.3g} & Minimize \\\\ \n'.format(matrix[4, 0], matrix[4, 1], matrix[4, 2], matrix[4, 3], matrix[4, 4], weights[4]))
        # Compute score
        scores = weights @ matrix_n
        f.write('Scores & {: 0.3g}& {: 0.3g} & {: 0.3g} & {: 0.3g} & {: 0.3g} & & \\\\ \n'.format(scores[0], scores[1], scores[2], scores[3], scores[4]))
        # Write rank
        rank = scipy.stats.rankdata(-scores, method='dense')
        f.write('Rank & {} & {} & {} & {} & {} & & \\\\'.format(rank[0], rank[1], rank[2], rank[3], rank[4]))