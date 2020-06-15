from com_func import *
from safe_cont import *
from perform_cont import *

def RTA_Gate(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, obstacle_list, target, current_mode, safe_complete, cur_wpt):
        #Parameters:
        obs_to_consider = 2 #Number of obstacles to consider in avoidance
        tol = 2 #Time (seconds) tolerance to avoid obstacle (3 means should always have a 3 second buffer between vehicle and obstacle)
        violation = 0 #Is a future collision detected?

        obs_total = np.size(obstacle_list, 1)
        num_obs = max(obs_to_consider, obs_total)  #Number of obstacles to care about (set to 2 right now, can adjust)

        obstacles = get_closest_obstacle(vehicle_x, vehicle_y, obstacle_list, num_obs)

        #Here is the basic boundary check based on current position and direction of vehicle. This will be modified with something fancier
        speed = math.sqrt(math.pow(vehicle_xdot,2) + math.pow(vehicle_ydot, 2))
        heading = math.atan2(vehicle_ydot, vehicle_xdot)

        if heading < 0:
            heading = 2*math.pi + heading

        d = max(speed*tol, 1) #distance separation required, should be at least 1 m


        for i in range(0, num_obs):

            rel_position_x = vehicle_x - obstacles[i][0]
            rel_position_y = vehicle_y - obstacles[i][1]
            obs_heading = math.atan2(-rel_position_y, -rel_position_x)
            if obs_heading < 0:
                obs_heading = 2*math.pi + obs_heading

            d_act = math.sqrt(math.pow(rel_position_x, 2) + math.pow(rel_position_y, 2))

            #Check if vehicle is currently maintaining a "tol" second buffer
            dist_check_1 = d_act - d

            #Obstacle is roughly in the current path of the vehicle
            if abs(heading - obs_heading) <= math.pi/6 and dist_check_1 <= 0:
                violation = 1
                collision_obstacle = i

            #Decide which waypoint to output
            if violation == 0 and current_mode == 0:
                wpt_performance = Performance_Controller(vehicle_x, vehicle_y, target)
                #This would be reinforcement learning controller. We can replace with something else for now.
                return wpt_performance, current_mode, safe_complete

            elif violation == 1 and current_mode == 0:
                #STart safe maneuver, reset current_mode, start timer
                current_mode = 1
                safe_complete = 0
                start_safe = 1
                wpt_safe_list = "global"
                Safe_Controller_Init(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, target, obstacle_list[i])
                return wpt_safe_list[0],  current_mode, safe_complete

            elif current_mode == 1 and safe_complete == 0:
                #Continue to execute trajectory until completed
                start_safe = 0
                wpt_safe, safe_complete = Safe_Controller(vehicle_x, vehicle_y, target, cur_wpt)

                return wpt_safe, current_mode, safe_complete

            else:
                #Safe maneuver has been completed, hopefully we don't switch right back to violation so give buffer?
                current_mode = 0
                wpt_performance = Performance_Controller(vehicle_x, vehicle_y, target)

                return wpt_performance, current_mode, safe_complete

if __name__ == '__main__':
    RTA_Gate()