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

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"

#define ROLL 0
#define PITCH 3.14
#define YAW 0

using namespace pcl;

class Cloud 
{
    private: 

        // members
        bool initialized;
        sensor_msgs::PointCloud2 cloud;
        ros::Publisher point_pub;
        ros::Publisher cloud_pub;
        ros::Subscriber cloud_sub;
        ros::Subscriber kill_sub;
        tf::TransformListener listener;

        // functions
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void kill_callback(const std_msgs::Bool& msg);
        void remove_outliers();
        void voxel_filter();
        void set_highest_point();
        PointCloud<PointXYZ> rotate_points(PointCloud<PointXYZ> old_cloud, float theta);
        geometry_msgs::Quaternion initialize_orientation();

    public:

        // members
        geometry_msgs::Point highest_point;
        bool done;

        // functions
        Cloud();
        Cloud(ros::NodeHandle handle);
        void get_refined_cloud();
};

#endif // PCL_CLASS_HPP

////////////////////////////////////////////////////////////////
