////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: pcl_class.hpp
//
// Purpose: Declares a class which handles operations on the 
// point clout created by the kinect
//
////////////////////////////////////////////////////////////////

#ifndef PCL_CLASS_HPP
#define PCL_CLASS_HPP

#include <cmath>
#include <iostream>
#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Cloud 
{
    private: 
        sensor_msgs::PointCloud2 cloud;
        geometry_msgs::Point highest_point;
        ros::Publisher point_pub;
        ros::Publisher cloud_pub;
        ros::Subscriber cloud_sub;
        tf::TransformListener listener;

    public:
        Cloud();
        Cloud(ros::NodeHandle handle);
        geometry_msgs::Point get_highest_point() { return this->highest_point; }
        void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void remove_outliers();
        void voxel_filter();
        void set_highest_point();
        void publish_point();
        pcl::PointCloud<pcl::PointXYZ> rotate_points(pcl::PointCloud<pcl::PointXYZ> old_cloud, float theta);
};

#endif // PCL_CLASS_HPP

////////////////////////////////////////////////////////////////
