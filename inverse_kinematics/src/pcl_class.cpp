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

////////////////////////////////////////////////////////////////

// Public Members

// DEFAULT CONSTRUCTOR
// does not do ros initialization
Cloud::Cloud() 
{
    this->initialized = false;
    this->done = false;
}

// CONSTRUCTOR
// does the ros initialization
Cloud::Cloud(ros::NodeHandle handle) 
{
    this->initialized = false;
    this->done = false;

    this->point_pub = handle.advertise<geometry_msgs::Point>("goal_point", 10);
    this->cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
	this->cloud_sub = handle.subscribe("camera/depth/points", 1, &Cloud::cloud_callback, this);
	this->kill_sub = handle.subscribe("kill_cloud", 1, &Cloud::kill_callback, this);

    while (!this->initialized) ros::spinOnce();
}

// GET REFINED CLOUD
// removes outliers & performs a voxel filter 
// on the cloud, then publishes the refined cloud
void Cloud::get_refined_cloud() 
{
    this->remove_outliers();
    this->voxel_filter();
    this->cloud_pub.publish(this->cloud);
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CLOUD CALLBACK FUNCTION
// gets the cloud as it's published
// sets the highest point of the cloud
void Cloud::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) 
{
    this->initialized = true;
    this->cloud = *msg;
    this->get_refined_cloud();
    this->set_highest_point();
}

// KILL CALLBACK FUNCTION
// this function wits for the kill command to be published
// to a certain topic; once it gets the signal, it will cause
// the pcl_node to commit suicide, exiting gracefully
void Cloud::kill_callback(const std_msgs::Bool& msg) 
{
    //ROS_INFO("Goodbye, cold cruel world... *dies*");
    this->done = msg.data;
}

// SET HIGHEST POINT FUNCTION
// finds the highest point in the cloud & publishes it
void Cloud::set_highest_point() 
{
    // Find the transformation between the point cloud and baxter
    tf::StampedTransform transform;
    try { this->listener.lookupTransform("/base", "/camera_depth_optical_frame", ros::Time(0), transform); }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // Convert the sensor_msgs::PointCloud2 to the PointCloud<PointXYZ> type
	PointCloud<PointXYZ> point_cloud;
	fromROSMsg(this->cloud, point_cloud);

    // Rotate the points in the cloud
	point_cloud = rotate_points(point_cloud, -0.94);

    // Get the point in the cloud with the smallest y-coordinate
	float smallest_point_value = INT_MAX;
	int index = -1;
	for (int i = 0; i < point_cloud.size(); i++) 
	{
		if (point_cloud.points[i].y < smallest_point_value) 
		{
			smallest_point_value = point_cloud.points[i].y;
			index = i;
		}
	}

    // Rotate the cloud back
	point_cloud = rotate_points(point_cloud, 0.94);

    // Get the highest point in terms of baxter instead of the point cloud
    geometry_msgs::PointStamped p_in, p_out;
    p_in.header.frame_id = "/camera_depth_optical_frame"; 
    p_in.point.x = point_cloud.points[index].x;
    p_in.point.y = point_cloud.points[index].y;
    p_in.point.z = point_cloud.points[index].z;
    this->listener.transformPoint("/base", p_in, p_out);

    // Set, print, & publish this point
	ROS_INFO("%d (%f, %f, %f)", index, p_out.point.x, p_out.point.y, p_out.point.z);
	this->highest_point = p_out.point;
    geometry_msgs::Point point;
    point.x = this->highest_point.x;
    point.y = this->highest_point.y;
    point.z = this->highest_point.z;
    this->point_pub.publish(point);

}

// REMOVE OUTLIERS FUNCTION
// removes points with neighbors that aren't 
// sufficiently close statistically speaking
void Cloud::remove_outliers() 
{
	PCLPointCloud2* cloud = new PCLPointCloud2;
	PCLPointCloud2ConstPtr cloud_ptr(cloud);
	PCLPointCloud2 filtered_cloud;

	pcl_conversions::toPCL(this->cloud, *cloud);

	StatisticalOutlierRemoval<PCLPointCloud2> sor;
  	sor.setInputCloud(cloud_ptr);
  	sor.setMeanK(10);
  	sor.setStddevMulThresh(1.0);
  	sor.filter(filtered_cloud);

	sensor_msgs::PointCloud2 output_cloud;
	pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

	this->cloud = output_cloud;
}

// VOXEL FILTER FUNCTION
// downsamples point cloud to make the number 
// of points that get published more reasonable
void Cloud::voxel_filter() 
{
	PCLPointCloud2* cloud = new PCLPointCloud2;
	PCLPointCloud2ConstPtr cloud_ptr(cloud);
	PCLPointCloud2 filtered_cloud;

	pcl_conversions::toPCL(this->cloud, *cloud);

	VoxelGrid<PCLPointCloud2> sor;
	sor.setInputCloud(cloud_ptr);
	sor.setLeafSize(0.1, 0.1, 0.1); // 10cm grid?
	sor.filter(filtered_cloud);

	sensor_msgs::PointCloud2 output_cloud;
	pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

	this->cloud = output_cloud;
}

// ROTATE POINTS FUNCTION
// hacky way to transform the points 
// by a certain amount without the transform
PointCloud<PointXYZ> Cloud::rotate_points(PointCloud<PointXYZ> old_cloud, float theta) 
{
  	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

  	PointCloud<PointXYZ>::Ptr transformed_cloud(new PointCloud<PointXYZ>());
  	transformPointCloud(old_cloud, *transformed_cloud, transform_2);

	return *transformed_cloud;
}

////////////////////////////////////////////////////////////////
