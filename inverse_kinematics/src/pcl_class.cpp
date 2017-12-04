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

// Default Constructor
// creates a default cloud
Cloud::Cloud() 
{
    this->is_done = false;
}

// Constructor
// creates a cloud with set publishers and subscribers
Cloud::Cloud(ros::NodeHandle handle) 
{
    this->is_done = false;
    this->point_pub = handle.advertise<geometry_msgs::Point>("highest_point", 10);
    this->cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
	this->cloud_sub = handle.subscribe("camera/depth/points", 1, &Cloud::callback, this);
}

// Callback
// Filters the cloud and sets the highest point of the cloud
void Cloud::callback(const sensor_msgs::PointCloud2ConstPtr& msg) 
{
    this->cloud = *msg;

    this->remove_outliers();
    this->voxel_filter();
    this->set_highest_point();

    this->cloud_pub.publish(this->cloud);
    this->publish_point();
        
    this->cloud_sub.shutdown();
}

// Publish Point
// publishes the highest point to a new topic
void Cloud::publish_point() 
{
    geometry_msgs::Point point;
    point.x = this->highest_point.x;
    point.y = this->highest_point.y;
    point.z = this->highest_point.z;
    this->point_pub.publish(point);
}

// Set Highest Poiunt Function
// finds the highest point in the cloud
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

    // Convert the sensor_msgs::PointCloud2 to the pcl::PointCloud<pclPointXYZ type>
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	pcl::fromROSMsg(this->cloud, point_cloud);

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

    // Set and print this point
	//ROS_INFO("%d (%f, %f, %f)", index, p_out.point.x, p_out.point.y, p_out.point.z);
	this->highest_point = p_out.point;
}

// Statistical Outlier Removal Function
// removes points with neighbors that arent sufficiently close
void Cloud::remove_outliers() 
{
	// Mke containers for the existing and new clouds to go in
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
	pcl::PCLPointCloud2 filtered_cloud;

	// Convert the sensor_msgs::PointCloud2 to a pcl::PointCloud2
	pcl_conversions::toPCL(this->cloud, *cloud);

	// Do the thing
	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  	sor.setInputCloud(cloud_ptr);
  	sor.setMeanK(10);
  	sor.setStddevMulThresh(1.0);
  	sor.filter(filtered_cloud);

	// Convert back to sensor_msgs::PointCloud2 
	sensor_msgs::PointCloud2 output_cloud;
	pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

	// Return the new, filtered, output cloud
	this->cloud = output_cloud;
}

// Voxel Filter Function
// downsamples point cloud to make #points more reasonable
void Cloud::voxel_filter() 
{
	// Mke containers for the existing and new clouds to go in
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
	pcl::PCLPointCloud2 filtered_cloud;

	// Convert the sensor_msgs::PointCloud2 to a pcl::PointCloud2
	pcl_conversions::toPCL(this->cloud, *cloud);

	// Do the thing
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_ptr);
	sor.setLeafSize(0.1, 0.1, 0.1);
	sor.filter(filtered_cloud);

	// Convert back to sensor_msgs::PointCloud2 
	sensor_msgs::PointCloud2 output_cloud;
	pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

	// Return the new, filtered, output cloud
	this->cloud = output_cloud;
}

// Rotate Points Function
// hacky way to transform the points without tf
pcl::PointCloud<pcl::PointXYZ> Cloud::rotate_points(pcl::PointCloud<pcl::PointXYZ> old_cloud, float theta) 
{
  	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

  	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  	pcl::transformPointCloud(old_cloud, *transformed_cloud, transform_2);

	return *transformed_cloud;
}

////////////////////////////////////////////////////////////////
