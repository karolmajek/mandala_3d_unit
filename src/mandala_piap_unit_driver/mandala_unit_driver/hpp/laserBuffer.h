#ifndef LASER_BUFFER_HPP
#define LASER_BUFFER_HPP
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "point_types.h"
#include <list>
#include <mutex>
const float VELODYNE_BUFFER_SECS = 5;
const float ANGLE_BUFFER_SECS = 5;
struct laserBufferChunk
{
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> chunk_pointcloud;
	double odometry_roll;
	double odometry_pitch;
	double odometry_yaw;
	double odometry_x;
	double odometry_y;
	double odometry_z;

};
struct laserBuffer {

	int laserType;
	std::list<laserBufferChunk> data;
	ros::Duration getBufferDurtation();
	void addData(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pc,
			double odometry_x,
			double odometry_y,
			double odometry_z,
			double odometry_roll,
			double odometry_pitch,
			double odometry_yaw);
};
class angleBuffer {
public:
	void insertMeasurment(double ts, double angle);

	std::map<double, double>::iterator findClosestAngleIt(double key);

	void findClosestAngle(double key, double &err, double &angle);

	double getBufferLenSec()
	{
		return angleBufferLenSecs;
	}
private:
	std::map<double, double> angleBuffer;
	double angleBufferLenSecs;
};

#endif
