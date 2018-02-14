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
    this->done_ = false;
}

// CONSTRUCTOR
// does the ros initialization
Cloud::Cloud(ros::NodeHandle handle) 
{
    this->initialized = false;
    this->done_ = false;

    this->point_pub = handle.advertise<geometry_msgs::Pose>("/right/goal_point", 10);
    this->cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
	this->cloud_sub = handle.subscribe("camera/depth/points", 1, &Cloud::cloud_callback, this);
	this->kill_sub = handle.subscribe("kill_cloud", 1, &Cloud::kill_callback, this);

    while (!this->initialized) ros::spinOnce();
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
    this->done_ = msg.data;
}

// GET REFINED CLOUD
// removes outliers & performs a voxel filter 
// on the cloud, then publishes the refined cloud
void Cloud::get_refined_cloud() 
{
	PointCloud<PointXYZ> point_cloud;
	fromROSMsg(this->cloud, point_cloud);
	point_cloud = rotate_points(point_cloud, -0.94);
    point_cloud = passthrough_filter(point_cloud);
	point_cloud = rotate_points(point_cloud, 0.94);
    toROSMsg(point_cloud, this->cloud);

    this->remove_outliers();
    this->voxel_filter();
    this->cloud_pub.publish(this->cloud);
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

    // Set & print the point
	ROS_INFO("\tcube found in pcl as point %d(%f, %f, %f)", index, p_out.point.x, p_out.point.y, p_out.point.z);
    p_out.point.x+= 0.05;
    p_out.point.z = 0.10;
	ROS_INFO("\tpublishing edited point (%f, %f, %f)", p_out.point.x, p_out.point.y, p_out.point.z);
	this->highest_point = p_out.point;

    // Publish the pose
    geometry_msgs::Pose pose;
    pose.position = this->highest_point;
    pose.orientation = this->initialize_orientation();
    this->point_pub.publish(pose);
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

// PASSTHROUGH FUNCTION
// removes all points not within a 
// specified volume in the point cloud
PointCloud<PointXYZ> Cloud::passthrough_filter(PointCloud<PointXYZ> input_cloud) 
{
    // variable declarations
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = input_cloud;
 
    // do the filtering on the x (left-right in reference to baxter)
    pcl::PassThrough<pcl::PointXYZ> x_pass;
    x_pass.setInputCloud (cloud);
    x_pass.setFilterFieldName ("x");
    x_pass.setFilterLimits (-1.0, 1.0);
    x_pass.filter (*cloud_filtered);
 
    // do the filtering on the y (up-down in reference to baxter)
    pcl::PassThrough<pcl::PointXYZ> y_pass;
    y_pass.setInputCloud (cloud_filtered);
    y_pass.setFilterFieldName ("y");
    y_pass.setFilterLimits (0.25, 0.75);
    y_pass.filter (*cloud_filtered);

    // do the filtering on the z (close-far in reference to baxter)
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud (cloud_filtered);
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits (0.0, 1.0);
    z_pass.filter (*cloud_filtered);
 
    return *cloud_filtered;
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
	sor.setLeafSize(0.05, 0.05, 0.05); // 5cm grid
	sor.filter(filtered_cloud);

	sensor_msgs::PointCloud2 output_cloud;
	pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

	this->cloud = output_cloud;
}

// INITIALIZE ORIENTATION
// initialize the orientation aspect of the pose
// such that the grippers are pointing straight down
geometry_msgs::Quaternion Cloud::initialize_orientation() 
{
	geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 1.0; // original 1
    q.z = 0.0;
    q.w = 0.0;

    return q;
}

////////////////////////////////////////////////////////////////
