////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: ik_client_node.hpp
//
// Description: This node handles the iks requests
//
//////////////////////////////////////////////////////////////

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

#include "endpoint_class.hpp"

#include <cstdlib>
#include <string>
#include <iostream>
 
#define DUMMY_X 0.6569
#define DUMMY_Y -0.8525
#define DUMMY_Z 0.0388

geometry_msgs::PoseStamped get_desired_pose(geometry_msgs::Point new_point);

using std::string;
using std::cout;
using std::endl;

int main(int argc, char **argv)
{
        // Initialize ros
        ros::init(argc, argv, "ik_client_node");
        ros::NodeHandle nh;
       
        // Create a joint publisher to send the arm to the desired point
        ros::Publisher joint_pub = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 10);
        ros::Rate loop_rate(10);

        // Create the client 
        const string service_name = "ExternalTools/right/PositionKinematicsNode/IKService";
        ros::ServiceClient client = nh.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name);

        // Get the endpoint setup
        Endpoint point(nh);
        geometry_msgs::PoseStamped pose = get_desired_pose(point.get_point());
	    ROS_INFO("(%f, %f, %f)", point.get_point().x, point.get_point().y, point.get_point().z);

        // Make the service request
        baxter_core_msgs::SolvePositionIK service;
        service.request.pose_stamp.resize(1);
        service.request.pose_stamp[0] = pose;
        service.request.seed_mode = 2;

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

        // Put the request's solved joint states into a variable
        sensor_msgs::JointState solved_state;
        solved_state = service.response.joints[0];

        // Set the solved states to the new command message
        baxter_core_msgs::JointCommand msg;
        msg.command.resize(solved_state.name.size());
        msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        for (int i = 0; i < solved_state.name.size(); i++) 
        {
            msg.names.push_back(solved_state.name[i]);
            msg.command[i] = solved_state.position[i];
        }
        
        while (ros::ok()) 
        {
            // Publish the solved positions to the joints
            joint_pub.publish(msg);

            // Spin & sleep
            ros::spinOnce();
            loop_rate.sleep();
        }
        
        return 0;
}

// GET DESIRED POSE
// Make the stamped_pose object; refactored from main
geometry_msgs::PoseStamped get_desired_pose(geometry_msgs::Point new_point) 
{
        // Make a header for the pose_stamped
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        header.frame_id = "base";
        
        // Make the point for the pose
        geometry_msgs::Point point;
        point.x = new_point.x; 
        point.y = new_point.y;
        point.z = new_point.z;
       
        // Make the quaternion for the pose 
        geometry_msgs::Quaternion quaternion;
        quaternion.x = -0.3000;
        quaternion.y = 0.9538;
        quaternion.z = 0.0017;
        quaternion.w = -0.0005;

        // Make the pose fot the pose_stamped
        geometry_msgs::Pose pose;
        pose.position = point;
        pose.orientation = quaternion;

        // Make the stamped pose
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.pose = pose;

        return pose_stamped;
}

