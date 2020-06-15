import numpy as np
import math

# This function takes as input the (x,y) position of the vehicle, the list of the center (x,y) position of all obstacles
# (assumed known) and the number of nearest obstacles to return, in order of proximity. It outputs the centers of the top num_obs obstacles
def get_closest_obstacle (vehicle_x, vehicle_y, obstacle_list, num_obs):

    obs_total = np.size(obstacle_list, 1) # obstacle list is [[x1 y1], [x2 y2], ...]
    distances = np.zeros(obs_total)
    closest_obs = np.zeros((num_obs, 2))

    for i in range(0, obs_total):
        distances[i] = distance(obstacle_list[i][0], obstacle_list[i][1], vehicle_x, vehicle_y)

    sorted_distances = np.argsort(distances)
    np.flipud(sorted_distances)

    for i in range(0, max(obs_total, num_obs)):
        closest_obs[i] = obstacle_list[sorted_distances[i]]

    return closest_obs


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def find_nearest_2D(array, value):
    array = np.asarray(array)
    dif = np.abs(np.subtract(array, value))
    ind = np.argmin(dif, axis=1)
    return ind[0] #This assumes that the min values are in the same row, which they should be for the scenario we care about.

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


