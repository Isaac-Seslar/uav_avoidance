#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



int main(int argc, char **argv)
{   
    int s;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }




    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    geometry_msgs::PoseStamped pose;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    
    // Takeoff to an altitude of 2 meters with 10 second timer
    ROS_INFO("Takeoff initiatied...");
    for( int i = 100; i > 0; --i ){
        
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        local_pos_pub.publish(pose);
        
        s = 1;

        ros::spinOnce();
        rate.sleep();
    }

    // while(ros::ok() && current_state.connected)

    // First setpoint
    if(s == 1){

        pose.pose.position.x = 0;
        pose.pose.position.y = 4;
        pose.pose.position.z = 2;
        ROS_INFO("Going to first setpoint...");
        s = 2;

        for( int i = 100; i > 0; --i){


            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }

    if(s == 2){
        pose.pose.position.x = 4;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        ROS_INFO("Going to second setpoint...");

        for( int i = 100; i > 0; --i){


            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }


// mavros_msgs::CommandTOL land_cmd;
// land_cmd.request.yaw = 0;
// land_cmd.request.latitude = 0;
// land_cmd.request.longitude = 0;
// land_cmd.request.altitude = 0;
    
}