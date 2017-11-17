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
        /*geometry_msgs::Point p;
        p.x =  0.331; // 0.754; 
        p.y = -0.114; // -0.284; 
        p.z =  0.844; // 0.086; */
        geometry_msgs::PoseStamped pose = get_desired_pose(point.get_point());
	    //ROS_INFO("(%f, %f, %f)", p.x, p.y, p.z); 
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
        point.z = 0.15;
       
        // The following math functions take a Roll, Pitch and Yaw and convert them to the XYZW for orientation.
        // These Roll, Pitch and Yaw values correspond to the hand pointing straight down.
        const double ROLL = 0, PITCH = 3.14, YAW = 0; // yaw should change
 
        double mathc1 = cos(PITCH);
        double maths1 = sin(PITCH);
        double mathc2 = cos(YAW);
        double maths2 = sin(YAW);
        double mathc3 = cos(ROLL);
        double maths3 = sin(ROLL);
                                                                     
        double oriw = sqrt(1.0 + mathc1 * mathc2 + mathc1 * mathc3 - maths1 * maths2 * maths3 + mathc2 * mathc3) / 2.0;
        double oriw4 = (4.0 * oriw);
        
        double orix = (mathc2 * maths3 + mathc1 * maths3 + maths1 * maths2 * mathc3) / oriw4;
        double oriy = (maths1 * mathc2 + maths1 * mathc3 + mathc1 * maths2 * maths3) / oriw4;
        double oriz = (-maths1 * maths3 + mathc1 * maths2 * mathc3 + maths2) / oriw4;

        // Make the quaternion for the pose 
        geometry_msgs::Quaternion quaternion;
        quaternion.x = orix; 
        quaternion.y = oriy;
        quaternion.z = oriz;
        quaternion.w = oriw;

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

