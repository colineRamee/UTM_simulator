from numpy import pi
import numpy as np
import math
import os


def clamp(mini, maxi, val):
    """Clamp the value between min and max"""
    return min(maxi, max(mini, val))


def angle_diff(a, b):
    """a-b between -pi and pi"""
    return ((a - b + pi) % (2 * pi)) - pi


def get_angle(a, b):
    """Get angle between two vectors a and b"""
    return np.math.atan2(np.linalg.det([a, b]), np.dot(a, b))


def get_times_agents_at_distance(delta_p, delta_v, distance):
    # delta_p difference in position (at a given time),
    # delta_v difference in velocity,
    # distance, solve for time such as the distance between the two agents at that time is equal to that value
    # return the list of times where the agents are at distance, returns None if no solution
    a = np.dot(delta_v, delta_v)
    b = 2 * np.dot(delta_v, delta_p)
    c = np.dot(delta_p, delta_p) - distance ** 2
    if a == 0:
        if c == 0:
            return [0]
        else:
            return None
    delta = b ** 2 - 4 * a * c
    if delta > 0:
        sqrt_delta = math.sqrt(delta)
        t1 = (-b - sqrt_delta) / (2 * a)
        t2 = (-b + sqrt_delta) / (2 * a)
        return [t1, t2]
    elif delta == 0:
        return [-b / (2 * a)]
    else:
        return None


def get_trajectory_circle_intersection(center, radius, start, velocity):
    """ Returns the time and location at which an agent starting at start and traveling at velocity will intersect the circle (center, radius)
        center, start and velocity must be numpy 2D array, units must be consistent (meters, meters/s) """
    d = start - center
    # Solving ||start+t*velocity-center|| = radius
    # Squared to get quadratic formula: a x**2 + 2b x + c = 0
    c = np.dot(d, d) - radius ** 2
    b = np.dot(d, velocity)
    a = np.dot(velocity, velocity)
    delta = b ** 2 - a * c
    if delta > 0 and a != 0:
        t1 = (-b - math.sqrt(delta)) / a
        x1 = start + t1 * velocity
        t2 = (-b + math.sqrt(delta)) / a
        x2 = start + t2 * velocity
        return [t1, t2], [x1, x2]
    if delta == 0 and a != 0:
        t = -b / a
        x = start + t * velocity
        return [t], [x]
    if a == 0:
        if np.linalg.norm(d) == radius:
            return [0], [start]
    return None, None


class MyLogger:
    """ Due to stdout stuff on the cluster either print to console or to a file"""

    def __init__(self, filename=None):
        self.filename = filename
        if self.filename is not None:
            with open(self.filename, 'w') as f:
                f.write('')

    def log(self, my_string):
        print(my_string)
        if self.filename is not None:
            with open(self.filename, 'a') as f:
                f.write(my_string + '\n')

    def remove_logfile(self):
        if self.filename is not None:
            os.remove(self.filename)
