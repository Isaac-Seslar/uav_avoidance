import numpy as np
import math
from com_func import *

def Performance_Controller(vehicle_x, vehicle_y, target):
    # Compute a line from vehicle to target
    slope = (target[1] - vehicle_y)/(target[0] - vehicle_x)

    direction = np.sign(target[0] - vehicle_x)

    x_next = vehicle_x + direction*0.1
    if direction > 0 and x_next < target[0]:
        x_dif = 0.1
    elif direction > 0 and x_next >= target[0]:
        x_dif = target[0] - vehicle_x
    elif direction < 0 and x_next > target[0]:
        x_dif = -0.1
    else:
        x_dif = target[0] - vehicle_x

    y_next = vehicle_y + slope*x_dif

    return np.array([x_next, y_next])

if __name__ == '__main__':
    Performance_Controller()