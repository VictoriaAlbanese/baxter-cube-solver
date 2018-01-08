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
#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"

using namespace pcl;

class Cloud 
{
    private: 

        // members
        bool initialized;
        bool done_;
        sensor_msgs::PointCloud2 cloud;
        ros::Publisher point_pub;
        ros::Publisher cloud_pub;
        ros::Subscriber cloud_sub;
        ros::Subscriber kill_sub;
        tf::TransformListener listener;

        // functions
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void kill_callback(const std_msgs::Bool& msg);
        void get_refined_cloud();
        void set_highest_point();
        void remove_outliers();
        void voxel_filter();
        PointCloud<PointXYZ> passthrough_filter(PointCloud<PointXYZ> input_cloud);
        PointCloud<PointXYZ> rotate_points(PointCloud<PointXYZ> old_cloud, float theta);
        geometry_msgs::Quaternion initialize_orientation();

    public:

        // members
        geometry_msgs::Point highest_point;

        // functions
        Cloud();
        Cloud(ros::NodeHandle handle);
        bool done() { return this->done_; }
};

#endif // PCL_CLASS_HPP

////////////////////////////////////////////////////////////////
