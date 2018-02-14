////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: ik_client_node.hpp
//
// Description: This node handles the iks requests
//
//////////////////////////////////////////////////////////////

#include <cstdlib>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/SolvePositionIKRequest.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
 
#define LEFT 0
#define RIGHT 1

#define R_AWAY 0.0
#define R_HOLDING_SOFT 0.03
#define R_HOLDING_FIRM 0.05
#define L_AWAY 0.0
#define L_HOLDING_SOFT -0.03
#define L_HOLDING_FIRM -0.05


sensor_msgs::JointState get_iks(ros::ServiceClient client, baxter_core_msgs::SolvePositionIK service);
baxter_core_msgs::SolvePositionIK make_service_request(ros::NodeHandle handle, bool side, float action); 
geometry_msgs::PoseStamped get_pose(bool side, float action);

using std::string;

int main(int argc, char **argv)
{
        // Initialize ros
        ros::init(argc, argv, "ik_client_node");
        ros::NodeHandle nh;
        ros::Rate loop_rate(10);
      
        // Setup the left arm
        ros::Publisher joint_pub_left = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
        const string left_name = "ExternalTools/left/PositionKinematicsNode/IKService";
        ros::ServiceClient client_left = nh.serviceClient<baxter_core_msgs::SolvePositionIK>(left_name);

        // Setup the right arm 
        ros::Publisher joint_pub_right = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 10);
        const string right_name = "ExternalTools/right/PositionKinematicsNode/IKService";
        ros::ServiceClient client_right = nh.serviceClient<baxter_core_msgs::SolvePositionIK>(right_name);

        // Variables
        baxter_core_msgs::SolvePositionIK service_left, service_right;
        sensor_msgs::JointState solved_state_left, solved_state_right;
        baxter_core_msgs::JointCommand left_away, left_holding_soft, left_holding_firm;
        baxter_core_msgs::JointCommand right_away, right_holding_soft, right_holding_firm;

        // Bring the right arm to the away position
        /*service_right = make_service_request(nh, RIGHT, R_AWAY);
        solved_state_right = get_iks(client_right, service_right);
        right_away.command.resize(solved_state_right.name.size());
        right_away.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state_right.name.size(); i++) 
        {
            right_away.names.push_back(solved_state_right.name[i]);
            right_away.command[i] = solved_state_right.position[i];
        }*/

        // Bring the right arm to the firm holding position
        service_right = make_service_request(nh, RIGHT, R_HOLDING_FIRM);
        solved_state_right = get_iks(client_right, service_right);
        right_holding_firm.command.resize(solved_state_right.name.size());
        right_holding_firm.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state_right.name.size(); i++) 
        {
            right_holding_firm.names.push_back(solved_state_right.name[i]);
            right_holding_firm.command[i] = solved_state_right.position[i];
        }
    
        // Bring the right arm to the soft holding position
        /*service_right = make_service_request(nh, RIGHT, R_HOLDING_SOFT);
        solved_state_right = get_iks(client_right, service_right);
        right_holding_soft.command.resize(solved_state_right.name.size());
        right_holding_soft.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state_right.name.size(); i++) 
        {
            right_holding_soft.names.push_back(solved_state_right.name[i]);
            right_holding_soft.command[i] = solved_state_right.position[i];
        }*/

        // Bring the left arm to the away position
        service_left = make_service_request(nh, LEFT, L_AWAY);
        solved_state_left = get_iks(client_left, service_left);
        left_away.command.resize(solved_state_left.name.size());
        left_away.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state_left.name.size(); i++) 
        {
            left_away.names.push_back(solved_state_left.name[i]);
            left_away.command[i] = solved_state_left.position[i];
        }
        
        // Bring the left arm to the soft holding position
        /*service_left = make_service_request(nh, LEFT, L_HOLDING_SOFT);
        solved_state_left = get_iks(client_left, service_left);
        left_holding_soft.command.resize(solved_state_left.name.size());
        left_holding_soft.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state_left.name.size(); i++) 
        {
            left_holding_soft.names.push_back(solved_state_left.name[i]);
            left_holding_soft.command[i] = solved_state_left.position[i];
        }

        // Bring the left arm to the firm holding position
        service_left = make_service_request(nh, LEFT, L_HOLDING_FIRM);
        solved_state_left = get_iks(client_left, service_left);
        left_holding_firm.command.resize(solved_state_left.name.size());
        left_holding_firm.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state_left.name.size(); i++) 
        {
            left_holding_firm.names.push_back(solved_state_left.name[i]);
            left_holding_firm.command[i] = solved_state_left.position[i];
        }
        */
        // Publish the solved positions to the joints
        while (ros::ok()) 
        {
            joint_pub_left.publish(left_away);
            joint_pub_right.publish(right_holding_firm);

            ros::spinOnce();
            loop_rate.sleep();
        }
        
        return 0;
}

// GET IKS
// Calls the service and gets the iks, or errors if there is none
sensor_msgs::JointState get_iks(ros::ServiceClient client, baxter_core_msgs::SolvePositionIK service) 
{
        // Report if the call to the service fails and exit
        if (!client.call(service))
        {
            ROS_ERROR("Failure: unable to call service");
            exit(1);
        }

        // Report if there is no ik solution and exit
        if (!service.response.isValid[0]) 
        {
            ROS_ERROR("Failure: no valid ik joint solution");
            exit(1);
        }
        ROS_INFO("SUCCESS!!");
        return service.response.joints[0];
}

// MAKE SERVICE REQUEST
// Makes the service request by setting up the endpoint
baxter_core_msgs::SolvePositionIK make_service_request(ros::NodeHandle handle, bool side, float action) 
{
        // Get the endpoint setup
        geometry_msgs::PoseStamped pose = get_pose(side, action);
	    ROS_INFO("(%f, %f, %f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        // Make the service request
        baxter_core_msgs::SolvePositionIK service;
        service.request.pose_stamp.resize(1);
        service.request.pose_stamp[0] = pose;
        service.request.seed_mode = 2;

        return service;
} 

// GET DESIRED POSE
// Make the stamped_pose object; refactored from main
geometry_msgs::PoseStamped get_pose(bool side, float action) 
{
        // Make a header for the pose_stamped
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        header.frame_id = "base";
        
        // Make the point for the pose
        geometry_msgs::Point point;
        point.x = 0.567;  
        point.y = action; 
        point.z = 0.637; 

        // Make the quaternion for the pose 
        geometry_msgs::Quaternion quaternion;
        quaternion.x =  0.000;  
        quaternion.y = -0.707;  
        quaternion.z = -0.707;
        quaternion.w =  0.000;
        if (side == LEFT) quaternion.y*= -1;             

        // Make the pose for the pose_stamped
        geometry_msgs::Pose pose;
        pose.position = point;
        pose.orientation = quaternion;

        // Make the stamped pose
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.pose = pose;

        return pose_stamped;
}

