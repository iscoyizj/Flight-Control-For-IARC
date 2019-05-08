/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>

using namespace std;

mavros_msgs::State current_state;
geometry_msgs::Pose local_pose;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void get_local_pose(const nav_msgs::Odometry::ConstPtr& msg){
    local_pose = msg->pose.pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/global_position/local", 10, get_local_pose);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geometry_msgs::PoseStamped pose_list[4];
    geometry_msgs::PoseStamped pose0;
	pose0.header.frame_id = "map";
    pose0.pose.position.x = 0;
    pose0.pose.position.y = 0;
    pose0.pose.position.z = 2;
    geometry_msgs::PoseStamped pose1;
	pose1.header.frame_id = "map";
    pose1.pose.position.x = 2;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 2;
    geometry_msgs::PoseStamped pose2;
	pose2.header.frame_id = "map";
    pose2.pose.position.x = 2;
    pose2.pose.position.y = 2;
    pose2.pose.position.z = 2;
    geometry_msgs::PoseStamped pose3;
	pose3.header.frame_id = "map";
    pose3.pose.position.x = 0;
    pose3.pose.position.y = 2;
    pose3.pose.position.z = 2;
    pose_list[0] = pose0;
    pose_list[1] = pose1;
    pose_list[2] = pose2;
    pose_list[3] = pose3;

    int current_target = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
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
        double distance = (local_pose.position.x-pose_list[current_target].pose.position.x)*(local_pose.position.x-pose_list[current_target].pose.position.x)
        + (local_pose.position.y-pose_list[current_target].pose.position.y)*(local_pose.position.y-pose_list[current_target].pose.position.y)
        + (local_pose.position.z-pose_list[current_target].pose.position.z)*(local_pose.position.z-pose_list[current_target].pose.position.z);
        if(distance < 0.1){
            current_target = (current_target+1)%4;
        }
        local_pos_pub.publish(pose_list[current_target]);     

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
