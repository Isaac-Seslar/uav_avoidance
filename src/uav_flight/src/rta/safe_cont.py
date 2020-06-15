import numpy as np
import math
from com_func import *
from scipy.io import loadmat
# from scipy import loadmat

def Safe_Controller (vehicle_x, vehicle_y,  target, cur_wpt):
    #Check current distance from cur_wpt, and distance to next waypoint, assign based on whether reached cur_wpt or not
    num_wpts = wpt_safe_list.shape[0]
    cur_wpt_ind = find_nearest_2D(wpt_safe_list, cur_wpt)
    direction = np.sign(vehicle_x - target[0])

    wpt_reached = distance(vehicle_x, vehicle_y, cur_wpt[0], cur_wpt[1])

    if wpt_reached < 0.1 & cur_wpt_ind < num_wpts - 1: #Move on to next waypoint

        next_wpt = wpt_safe_list[cur_wpt_ind+1]
        safe_complete = 0

    elif wpt_reached < 0.1 & cur_wpt_ind == num_wpts -1:
        next_wpt = cur_wpt
        safe_complete = 1

    #Next check if waypoint has been overshot, so find nearest waypoint that is closer to target in x-direction
    #If direction < 0, vehicle needs to move in pos x direction
    elif vehicle_x < cur_wpt[0] & direction < 0:
        next_wpt = cur_wpt
        safe_complete = 0
    elif vehicle_x > cur_wpt[0] & direction >=0:
        next_wpt = cur_wpt
        safe_complete = 0
    elif vehicle_x >= cur_wpt[0] & direction < 0:
        ind = cur_wpt_ind
        while ind < num_wpts & vehicle_x >= wpt_safe_list[ind]:
            ind += 1
        if ind == num_wpts:
            next_wpt = wpt_safe_list[ind-1]
            safe_complete = 1
        else:
            next_wpt = wpt_safe_list[ind]
            safe_complete = 0
    elif vehicle_x <= cur_wpt[0] & direction < 0:
        ind = cur_wpt_ind
        while ind < num_wpts & vehicle_x <= wpt_safe_list[ind]:
            ind +=1
        if ind == num_wpts:
            next_wpt = wpt_safe_list[ind-1]
            safe_complete = 1
        else:
            next_wpt = wpt_safe_list[ind]
            safe_complete = 0
    else:
        #Just in case we missed something, keep current waypoint
        next_wpt = cur_wpt
        safe_complete = 0

    return next_wpt, safe_complete


def Safe_Controller_Init (vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, target, obstacle):

    #Load mat file of safe radii around obstacle
    arcs = np.genfromtxt('~/irad_rta_2020/RTAFramework/ARCS.mat', delimiter=',')

    x_rel = vehicle_x - obstacle[0]
    y_rel = vehicle_y - obstacle[1]

    in_theta = math.atan2(y_rel, x_rel)

    #Get parameters for safe circular path around obstacle

    xtable = np.array([-4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5])
    ytable = np.array([-4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5])
    xVtable = np.array([-.45, -.35, -.25, -.15, -.05, .05, .15, .25, .35, .45])
    yVtable = np.array([-.45, -.35, -.25, -.15, -.05, .05, .15, .25, .35, .45])

    idx = find_nearest(xtable, x_rel)
    idy = find_nearest(ytable, y_rel)
    idxV = find_nearest(xVtable, vehicle_xdot)
    idyV = find_nearest(yVtable, vehicle_ydot)

    #r = arcs[idx][idy][idxV][idyV]

    x_ind = 11*(idxV) + idx
    y_ind = 11*idyV + idy
    r = arcs[x_ind][y_ind]

    out_x = target[0] - obstacle[0]
    out_y = target[1] - obstacle[1]
    out_theta = math.atan2(out_y, out_x)

    xCircEnd = r*math.cos(out_theta)
    yCircEnd = r*math.sin(out_theta)

    xFinal = xCircEnd + min(0.2*(target[0] - xCircEnd), 0.5)
    yFinal = yCircEnd + ((target[1] -  yCircEnd)/(target[0]-xCircEnd))*(xFinal - xCircEnd)


    xStart = r*math.cos(in_theta)
    yStart = r*math.sin(in_theta)

    omega = 0.1

    theta = in_theta - out_theta
    if in_theta >=0 and out_theta >= 0:
        if theta > 0:
            sn = 1
        else:
            sn = -1
    elif in_theta >= 0 and out_theta < 0:
        out_theta += 2*math.pi
        theta = in_theta - out_theta
        if abs(theta) < math.pi:
            sn = -1
        else:
            sn = 1
    elif in_theta < 0 and out_theta >=0:
        in_theta += 2*math.pi
        theta = in_theta - out_theta
        if theta < math.pi:
            sn = -1
        else:
            sn = 1
    else:
        if theta > 0:
            sn = 1
        else:
            sn = -1

    #Set trajectory waypoints using parameters r, sn, omega, theta, xFinal, yFinal:
    wpts_safe = np.array([xStart, yStart])
    theta = in_theta
    while theta <= out_theta:
        theta = in_theta + sn*omega
        x = r*math.cos(theta)
        y = r*math.sin(theta)
        wpts_safe = np.append(wpts_safe, [x, y])

    #Add final waypoint away from circle:
    wpt_safe_list = np.append(wpts_safe, [xFinal, yFinal])


if __name__ == '__main__':
    Safe_Controller()
    Safe_Controller_Init()