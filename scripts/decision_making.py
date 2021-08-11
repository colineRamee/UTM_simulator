import numpy as np


def TOPSIS(decision_matrix, weights, objectives, alternatives):
    np.set_printoptions(suppress=True)
    print(alternatives)
    print(decision_matrix)
    n = len(weights)  # Number of criterias
    m = len(alternatives)  # Number of alternatives
    R = decision_matrix  # Decision matrix
    normalized_R = np.zeros((n, m))
    # Step 1: normalize the decision matrix, and Step 2: apply weights, and Step 3: identify positive and negative ideal
    # Normalize the weights
    weights = weights / np.linalg.norm(weights)
    best = np.zeros(n)
    worst = np.zeros(n)
    for i in range(0, n):
        norm = np.linalg.norm(R[i, :])
        w_i = weights[i]
        obj_i = objectives[i]
        min_ = None
        max_ = None
        for j in range(0, m):
            val = w_i * R[i, j] / norm
            normalized_R[i, j] = val
            if min_ is None or val < min_:
                min_ = val
            if max_ is None or val > max_:
                max_ = val
        if obj_i == 'maximize':
            best[i] = max_
            worst[i] = min_
        elif obj_i == 'minimize':
            best[i] = min_
            worst[i] = max_

    # Step 4: Measure distance to positive and negative ideal solution
    scores = np.zeros(m)
    for j in range(0, m):
        pos = np.linalg.norm(normalized_R[:, j] - best)
        neg = np.linalg.norm(normalized_R[:, j] - worst)
        rel_distance = neg / (pos + neg)
        scores[j] = rel_distance

    # Sort from smallest to largest
    sorted_index = np.argsort(scores)
    results = []
    for i in range(0, m):
        index = sorted_index[-i - 1]
        results.append([alternatives[index], scores[index]])
    return results


def SAW(decision_matrix, weights, alternatives):
    np.set_printoptions(suppress=True)
    print(alternatives)
    print(decision_matrix)
    n = len(weights)  # Number of criterias
    m = len(alternatives)  # Number of alternatives
    R = decision_matrix  # Decision matrix
    normalized_R = np.zeros((n, m))
    for i in range(0, n):
        normalized_R[i, :] = R[i, :] / max(R[i, :])
    w = weights / sum(abs(weights))
    print(w)
    scores = w @ normalized_R
    sorted_index = np.argsort(scores)
    results = []
    for i in range(0, m):
        index = sorted_index[-i - 1]
        results.append([alternatives[index], scores[index]])
    return results
