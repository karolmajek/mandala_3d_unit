#include "laserBuffer.h"

ros::Duration laserBuffer::getBufferDurtation() {
	if (data.size() < 1)
		return ros::Duration(0);
	auto b = data.rbegin();
	auto e = data.begin();
	ros::Duration d;
	d.fromNSec(1000 * (e->chunk_pointcloud.header.stamp - b->chunk_pointcloud.header.stamp));
	return d;
}
void laserBuffer::addData(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pc,
		double odometry_x,
		double odometry_y,
		double odometry_z,
		double odometry_roll,
		double odometry_pitch,
		double odometry_yaw)
	{
		laserBufferChunk p ;
		p.chunk_pointcloud = pc;
		p.odometry_x = odometry_x;
		p.odometry_y = odometry_y;
		p.odometry_z = odometry_z;
		p.odometry_roll = odometry_roll;
		p.odometry_pitch = odometry_pitch;
		p.odometry_yaw = odometry_yaw;
		data.push_front(p);
	}
void angleBuffer::insertMeasurment(double ts, double angle) {
	auto it = angleBuffer.find(ts);
	if (it == angleBuffer.end()) {
		angleBuffer[ts] = angle;
		if (angleBuffer.size() > 2) {
			angleBufferLenSecs = angleBuffer.rbegin()->first
					- angleBuffer.begin()->first;

			ROS_INFO_THROTTLE(1, "Angle len : %f, count: %d",
					angleBufferLenSecs, angleBuffer.size());
			if (angleBufferLenSecs > 2*ANGLE_BUFFER_SECS) {
							angleBuffer.clear();
						}
			if (angleBufferLenSecs > ANGLE_BUFFER_SECS) {
				angleBuffer.erase(angleBuffer.begin()->first);
			}
			if (angleBuffer.size() > 1000) {
				angleBuffer.erase(angleBuffer.begin()->first);
			}
		}
	}
}
std::map<double, double>::iterator angleBuffer::findClosestAngleIt(double key) {
	if (angleBuffer.size() == 0) {
		return angleBuffer.end();
	}

	auto lower = angleBuffer.lower_bound(key);

	if (lower == angleBuffer.end()) // If none found, return the last one.
		return std::prev(lower);

	if (lower == angleBuffer.begin())
		return lower;

	// Check which one is closest.
	auto previous = std::prev(lower);
	if ((key - previous->first) < (lower->first - key))
		return previous;

	return lower;
}
void angleBuffer::findClosestAngle(double key, double &err, double &angle) {

	auto it = findClosestAngleIt(key);
	if (!(it == angleBuffer.end())) {
		err = fabs(it->first - key);
		angle = it->second;
	}

}
