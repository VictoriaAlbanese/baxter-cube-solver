////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: pcl_class.cpp
//
// Purpose: Implements a class which handles operations on the 
// point clout created by the kinect
//
////////////////////////////////////////////////////////////////

#include "pcl_class.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

void publish_markers(pcl::PointXYZ point, ros::NodeHandle handle); 

int main(int argc, char ** argv) 
{
	// Initialize ros
	ros::init(argc, argv, "plc_node");
	ros::NodeHandle nh;

    // Make the cloud
    Cloud cloud(nh);

    // Make the markers
    publish_markers(cloud.get_highest_point(), nh);

    // Spin
    ros::spin();
}

// PUBLISH MARKERS
// publishes a marker at the highest point
// and also one 0.1 meters (10cm) above that point
void publish_markers(pcl::PointXYZ point, ros::NodeHandle handle) 
{
    ros::Publisher pub_vis = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

    visualization_msgs::Marker marker;
    visualization_msgs::Marker above_marker;
    visualization_msgs::MarkerArray markerArray;

	marker.header.frame_id = "/camera_depth_optical_frame";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = point.x;
	marker.pose.position.y = point.y;
	marker.pose.position.z = point.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.9;
	marker.color.g = 0.0;
	marker.color.b = 0.9;

    marker = above_marker;
	above_marker.id = 2;
	above_marker.pose.position.y-= 0.1 * cos(0.94);
	above_marker.pose.position.z-= 0.1 * sin(0.94);
	above_marker.color.r = 0.0;
	above_marker.color.g = 1.0;
	above_marker.color.b = 0.0;

    markerArray.markers.resize(2);
	markerArray.markers[0] = marker;
	markerArray.markers[1] = above_marker;

	pub_vis.publish(markerArray);
}
