#pragma once

#include <iostream>
#include <thread>
#include <string>
#include <mutex>
#include <queue>

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "radar_gather/RadarScan.h"

using namespace std;
using namespace Eigen;

// structure for each pointcloud's scan
struct PointRADAR
{
	PCL_ADD_POINT4D;
	PCL_ADD_INTENSITY;
	double az;
	double vr;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointRADAR,
								(float, x, x)
								(float, y, y)
								(float, z, z)
								(float, intensity, intensity)
								(double, az, az)
								(double, vr, vr)
)

typedef pcl::PointCloud<PointRADAR> PointCloud;

// structure for each point's scan (including point_id)
struct PointScan
{
	PCL_ADD_POINT4D;
	PCL_ADD_INTENSITY;
    float range;
    float velocity;
    float bearing;
    uint16_t doppler_bin;
    uint16_t point_id;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointScan,
								(uint16_t, point_id, point_id)
								(float, x, x)
								(float, y, y)
								(float, z, z)
								(float, intensity, intensity)
								(float, range, range)
								(float, velocity, velocity)
)

class RadarG{
    public:
        RadarG(void);
        RadarG(int num_radar);
        ~RadarG(void);

        void ros_init(ros::NodeHandle &n);
        void point_cb(const radar_gather::RadarScan::ConstPtr& msg);
        void runOnce(void);

        queue<PointScan> q;
        queue<PointScan> p_data;

        queue<ros::Time> time_stamp;

        PointCloud temp_cloud;    
    private:
        int num_radar_ = 0;

        string str_point_;
        string str_cld_;

        ros::Subscriber sub_point_;
        ros::Publisher pub_cld_;

        mutex mtx1;
        mutex mtx2;
};