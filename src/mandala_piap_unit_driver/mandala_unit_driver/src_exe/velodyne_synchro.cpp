#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "point_types.h"
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>

#include "mandala_unit_driver/encoder_stamped.h"
#include <thread>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <chrono>
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

typedef velodyne_pointcloud::PointXYZIR inputPointType;
typedef velodyne_pointcloud::PointXYZIR outputPointType;
typedef velodyne_pointcloud::PointXYZIR point_time_type;
typedef std::deque<point_time_type> inputbuffertype;

int pubSub = 0;
const double INVALID_ANGLE = 10.0f;
const double INVALID_ANGLE_NEW = 11.0f;
const double INVALID_ANGLE_OLD = 12.0f;
const int ANGLE_BUFFER_LEN = 500;

class angleBufferElement {
public:
	angleBufferElement(double t, double a) {
		ang = a;
		tim = t;
	}
	double ang;
	double tim;
	bool operator <(const angleBufferElement& s) const {
		return (tim < s.tim);
	}
};

class angleBuffer {
	typedef std::deque<angleBufferElement> innerAngleBufferType;

public:
	void clearBuffer() {
		angleBufferLock.lock();
		buffer.clear();
		angleBufferLock.unlock();
	}

	angleBuffer() {
		lastTimeStamp = 0.0;
		firstTimeStamp = 0.0;
		d = 0;
	}

	double getAngle(double ts, bool &isOk) {
		isOk = false;

		float bestFit = 10;
		float bestAngle = INVALID_ANGLE;

		angleBufferLock.lock();
		innerAngleBufferType::iterator fit = buffer.end();
		float angle = INVALID_ANGLE;
		for (innerAngleBufferType::iterator it = buffer.begin();
				it != buffer.end(); it++) {
			float currentFit = fabs(it->tim - ts);
			if (currentFit < bestFit) {
				fit = it;
				bestFit = currentFit;
				bestAngle = it->ang;
			}
		}
		angleBufferLock.unlock();

		static int kk = 0;
		ROS_INFO_THROTTLE(1, "best Fit : %f", bestFit);
		double ang = bestAngle;
		if (bestFit < 0.05) {
			isOk = true;
		} else {
			//ROS_WARN ("besFit : %f", bestFit);
		}
		return ang;
	}

	void addPoint(double ts, float angle) {

		ts = fmod(ts, 3600.0);
		angleBufferLock.lock();
		buffer.push_back(angleBufferElement(ts, angle));

		lastTimeStamp = ts;
		if (getBufferCount() > ANGLE_BUFFER_LEN) {
			buffer.erase(buffer.begin());
		}

		firstTimeStamp = buffer.front().tim;
		lastTimeStamp = buffer.back().tim;
		angleBufferLock.unlock();

	}

	double getBufferLen() {
		return ((lastTimeStamp - firstTimeStamp));
	}
	size_t getBufferCount() {
		return buffer.size();
	}

	double getFirstTimeStamp() {
		return firstTimeStamp;
	}
	double getLastTimeStamp() {
		return lastTimeStamp;
	}

private:
	int d;
	float lastRequestedAngle;
	float lastTimeStamp;
	float firstTimeStamp;
	float len;
	innerAngleBufferType buffer;

	std::mutex angleBufferLock;

};

class velo_sync {
private:
	angleBuffer angBuff;
	int stopScanCount;
	void getTranslationRotationFromParams(float x, float y, float z, float yaw,
			float pitch, float roll, Eigen::Vector3f &t_m,
			Eigen::Quaternionf &t_q) {
		Eigen::Vector3f t_rot_euler;

		t_m.x() = x;
		t_m.y() = y;
		t_m.z() = z;

		t_rot_euler.x() = yaw;
		t_rot_euler.y() = pitch;
		t_rot_euler.z() = roll;

		t_q = Eigen::AngleAxisf(t_rot_euler[0], Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(t_rot_euler[1], Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(t_rot_euler[2], Eigen::Vector3f::UnitZ());

	}
	Eigen::Vector3f t_m;
	Eigen::Quaternionf t_q;

public:
	velo_sync() :
			n() {
		stopScanCount = 0;
		useSoftSync = false;
		double p[6];
		double laserHeight = 0;
		//		0.1627
		n.param<double>("lh", laserHeight, 0.1627);
		n.param<std::string>("tf_base", tf_base, "base");
		n.param<std::string>("tf_velodyne", tf_velodyne, "velodyne");
		n.param<double>("p0", p[0], 0);
		n.param<double>("p1", p[1], 0);
		n.param<double>("p2", p[2], 0);
		n.param<double>("p3", p[3], 0);
		n.param<double>("p4", p[4], 0);
		n.param<double>("p5", p[5], 0);
		n.param<bool>("useSoftSync", useSoftSync, false);
		n.param<double>("stopscanvel", v_stop_scan, 100.0);
		n.param<double>("nostopscanvel", v_non_stop_scan, 450.0);

		ROS_INFO_STREAM("useSoftSync :" << useSoftSync);
		ROS_INFO("tf_base : %s", tf_base.c_str());
		ROS_INFO("tf_velodyne : %s", tf_velodyne.c_str());

		getTranslationRotationFromParams(0, p[0], p[1], p[2], p[3], p[4], t_m,
				t_q);

		stopScan = false;
		// Create a ROS subscriber for the input point cloud
		sub1 = n.subscribe("velodyne_points", 1, &velo_sync::cloud_cb, this);
		sub2 = n.subscribe("unit_driver/current_angle", 1,
				&velo_sync::angle_cb, this);
		sub3 = n.subscribe("stopScanRequest", 1, &velo_sync::stop_scan_cb,
				this);

		tf::Quaternion q;
		tf::Transform transform2;
		transform2.setOrigin(tf::Vector3(laserHeight, 0, 0));
		q.setRPY(0, M_PI / 2, 0);
		transform2.setRotation(q);

		tf::Transform transform3;
		transform3.setOrigin(tf::Vector3(0, 0, 0));
		q.setRPY(0, 0, M_PI / 2);
		transform3.setRotation(q);

		tf::Transform transform4;
		transform4.setOrigin(tf::Vector3(t_m.x(), t_m.y(), t_m.z()));
		transform4.setRotation(
				tf::Quaternion(t_q.x(), t_q.y(), t_q.z(), t_q.w()));
		tg = transform2 * transform3 * transform4;

		pub = n.advertise<pcl::PointCloud<inputPointType> >("output", 1);

		pub2 = n.advertise<pcl::PointCloud<inputPointType> >("operational", 1);
		pub_stopScan_cloud = n.advertise<sensor_msgs::PointCloud2>(
				"stopScanOutput", 1);
		pub_stopScan_progress = n.advertise<std_msgs::Float32>(
				"stopScanProgress", 1);
		pub_stopScan_done = n.advertise<std_msgs::Bool>("done", 1);
		pub_camera_trigger = n.advertise<std_msgs::Bool>("camera/trig", 1);
		pub_scan_velocity = n.advertise<std_msgs::Float32>(
				"unit_driver/velocity", 1);
		pub_scan_marker_progress = n.advertise<visualization_msgs::MarkerArray>(
				"unit_driver/progress", 1);
		ros::Timer timer = n.createTimer(ros::Duration(0.150),
				&velo_sync::timer_cb, this);
		ros::Timer timer2 = n.createTimer(ros::Duration(0.05),
						&velo_sync::timer_send_angle_cb, this);
		numberOfChunks = 0;
		std::thread t(&velo_sync::syncThreadWorker, this);
		t.detach();

		ros::spin();
	}

	tf::Transform getTransformFromAngle(float angle) {

		tf::Quaternion q;
		//std::cout << "aaaa : " << client->ang << "\n";
		q.setRPY(0, 0, -angle);
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0, 0, 0));
		transform.setRotation(q);

		return transform * tg;

	}

	void stop_scan_cb(std_msgs::Bool b) {

		if (b.data) {
			stopScan_currentDistance = 0;
			stopScan = true;
			pub_camera_trigger.publish(b);
			ROS_INFO("starting aggregating stopscan");
			std_msgs::Float32 f;
			f.data = v_stop_scan;
			pub_scan_velocity.publish(f);
		}
	}
	int k = 0;
	void timer_send_angle_cb(const ros::TimerEvent&) {
		tf::Transform t = getTransformFromAngle(lastRecievedAngle);
		tf_br.sendTransform(tf::StampedTransform(t, ros::Time::now(), tf_base, tf_velodyne));
	}
	void timer_cb(const ros::TimerEvent&) {
		ROS_INFO_THROTTLE(1, "Number of chunks in buffer : %d",
							velodyneChunksCount);
		pcl::PointCloud<inputPointType>::Ptr resultToSend = nullptr;
		resultCloudLock.lock();
		float stopScan_currentDistanceLocal = stopScan_currentDistance;
		int result_size = resultCloud.size();
		if (result_size > 10000) {
			resultToSend = pcl::PointCloud<inputPointType>::Ptr(
					new pcl::PointCloud<inputPointType>);
			*resultToSend = resultCloud;
			resultCloud.clear();
		}
		resultCloudLock.unlock();

		if (resultToSend) {

			pcl_conversions::toPCL(ros::Time::now(),
					resultToSend->header.stamp);
			resultToSend->header.frame_id = tf_base;
			pub.publish(*resultToSend);
			if (pub2.getNumSubscribers() > 0) {
				pcl::PointCloud<outputPointType> out_operational;
				out_operational.reserve(0.15 * resultToSend->size());
				for (int i = 0; i < resultToSend->size(); i = i + 15) {
					inputPointType p = (*resultToSend)[i];
					float d = p.x * p.x	+ p.y * p.y;
					float z = p.z;
					if (d < 15 * 15) {
						if (z > -5 && z < 2.0) {
							out_operational.push_back(p);
						}
					}
				}
				pcl_conversions::toPCL(ros::Time::now(),
						out_operational.header.stamp);
				out_operational.header.frame_id = tf_base;

				pub2.publish(out_operational);
			}
		}



		if (stopScan) {

			std_msgs::Float32 f;
			f.data = 100.0 * stopScan_currentDistanceLocal / (2.1 * M_PI);
			if (f.data > 100)
				f.data = 100;
			if (pubSub >= 10 || f.data == 100) {
				std::stringstream ss;
				ss << int(f.data);
				pubSub = 0;
				pub_stopScan_progress.publish(f);
				visualization_msgs::MarkerArray arr;
				visualization_msgs::Marker m;
				m.action = visualization_msgs::Marker::MODIFY;
				m.id = 10001;
				m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				m.header.frame_id = tf_base;
				m.pose.position.z = 1;
				m.pose.orientation.w = 1;
				m.scale.x = 0.4;
				m.scale.y = 0.4;
				m.scale.z = 0.4;
				m.color.a = 1;
				m.color.r = 1;

				m.text = ss.str();
				arr.markers.push_back(m);
				pub_scan_marker_progress.publish(arr);
			}
			pubSub++;

		}
		if (stopScan && stopScan_currentDistance > 2.1 * M_PI) {
			sensor_msgs::PointCloud2 output_stopscan;
			resultCloudLock.lock();
			pcl::toROSMsg(out_pc_stop_scan, output_stopscan);
			out_pc_stop_scan.clear();
			resultCloudLock.unlock();
			output_stopscan.header.stamp = ros::Time::now();
			output_stopscan.header.frame_id =tf_base;
			output_stopscan.header.seq = stopScanCount;
			pub_stopScan_cloud.publish(output_stopscan);

			visualization_msgs::MarkerArray arr;
			visualization_msgs::Marker m;

			m.action = visualization_msgs::Marker::MODIFY;
			m.id = 10001;
			m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			m.header.frame_id = tf_base;
			m.pose.position.z = 1;
			m.pose.orientation.w = 1;
			m.scale.x = 1;
			m.scale.y = 1;
			m.scale.z = 1;
			m.color.a = 1;
			m.color.g = 1;

			m.text = "Done";
			arr.markers.push_back(m);
			pub_scan_marker_progress.publish(arr);

			stopScan = false;
			std_msgs::Bool b;
			b.data = true;
			pub_stopScan_done.publish(b);

			std_msgs::Float32 f;
			f.data = v_non_stop_scan;
			pub_scan_velocity.publish(f);
			stopScanCount++;
		}
	}
	void angle_cb(mandala_unit_driver::encoder_stamped e) {
		float currentEncoderTs = 0.000001 * e.timestamp;
		lastRecievedAngle = e.angle;
		angBuff.addPoint(currentEncoderTs, e.angle);

	}
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

		pcl::PointCloud<inputPointType> in_pc;
		pcl::fromROSMsg(*input, in_pc);
		int chunk_size = 100;
		std::deque<pcl::PointCloud<inputPointType>::Ptr> velodyneChunksLocal;
		for (int i = 0; i < in_pc.size(); i = i + chunk_size) {
			pcl::PointCloud<inputPointType>::Ptr chunk(
					new pcl::PointCloud<inputPointType>());
			chunk->reserve(chunk_size);
			for (int j = i; j < i + chunk_size; j++) {
				if (j < in_pc.size())
					chunk->push_back(in_pc[j]);
			}
			velodyneChunksLocal.push_back(chunk);

		}
		velodyneChunksLock.lock();
		velodyneChunks.insert(velodyneChunks.end(), velodyneChunksLocal.begin(),
				velodyneChunksLocal.end());
		velodyneChunksLock.unlock();
	}
	void syncThreadWorker() {
		while (ros::ok()) {
			pcl::PointCloud<inputPointType>::Ptr currentChunk = nullptr;
			velodyneChunksLock.lock();
			velodyneChunksCount = velodyneChunks.size();
			if (velodyneChunksCount > 0) {
				currentChunk = velodyneChunks.front();
				velodyneChunks.pop_front();
			}

			velodyneChunksLock.unlock();
			if (currentChunk == nullptr) {
				continue;
			}

			if (currentChunk->size() == 0) {
				continue;
			}
			float ts = (*currentChunk)[0].timestamp;

			float angle;
			bool isOk;
			angle = angBuff.getAngle(ts, isOk);

			tf::Transform tf_transformAngle = getTransformFromAngle(angle);

			if (isOk) {
				for (int i = 0; i < currentChunk->size(); i++) {

					tf::Vector3 p0((*currentChunk)[i].x, (*currentChunk)[i].y,
							(*currentChunk)[i].z);
					tf::Vector3 p1;
					p1 = tf_transformAngle * p0;

					(*currentChunk)[i].x = p1.x();
					(*currentChunk)[i].y = p1.y();
					(*currentChunk)[i].z = p1.z();

				}

				float dd = fabs(angle - lastAngle);

				resultCloudLock.lock();
				if (!std::isnan(dd) && dd < 0.1) {
					stopScan_currentDistance = stopScan_currentDistance + dd;
				}
				resultCloud += *currentChunk;
				if (stopScan)
					out_pc_stop_scan += *currentChunk;
				resultCloudLock.unlock();

				lastAngle = angle;
			}

		}
	}
private:
	float lastRecievedAngle;
	int velodyneChunksCount;
	int numberOfChunks;
	tf::TransformBroadcaster tf_br;

	std::thread syncThread;
	std::string tf_base;
	std::string tf_velodyne;
	std::mutex velodyneChunksLock;
	std::deque<pcl::PointCloud<inputPointType>::Ptr> velodyneChunks;
	std::mutex resultCloudLock;
	pcl::PointCloud<inputPointType> resultCloud;
	bool useSoftSync;
	tf::Transform tg;
	ros::NodeHandle n;

	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Subscriber sub3;

	ros::Publisher pub;

	ros::Publisher pub2;
	ros::Publisher pub_stopScan_cloud;
	ros::Publisher pub_stopScan_progress;
	ros::Publisher pub_stopScan_done;
	ros::Publisher pub_camera_trigger;
	ros::Publisher pub_scan_velocity;
	ros::Publisher pub_scan_marker_progress;

	double v_stop_scan;
	double v_non_stop_scan;

	float lastAngle;
	pcl::PointCloud<outputPointType> out_pc;
	pcl::PointCloud<outputPointType> out_pc_copy;
	pcl::PointCloud<outputPointType> out_pc_stop_scan;
	bool stopScan;
	float stopScan_currentDistance;

	inputbuffertype inputBuffer;
	tf::TransformBroadcaster* m3dRot;

};
int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "velodyne_sync");
	velo_sync();
}
