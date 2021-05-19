#include <iostream>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "mavlink.h"

#include "mavlink_client.h"
#include "mavlink_client_udp.h"
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>
#include <mandala_unit_driver/encoder_stamped.h>
#include <time.h>
#include <mandala_unit_msgs/cameraTrigger.h>
#include "roscpp/Logger.h"
mavlink_client * client = NULL;
float offset = 0;

ros::Publisher angPub;
ros::Publisher timeStampPub;
int speed = 0;
void velocityCallback(const std_msgs::Float32ConstPtr& msg)
{
    speed = msg->data;
}

bool triggerCamera(mandala_unit_msgs::cameraTrigger::Request  &req,
         mandala_unit_msgs::cameraTrigger::Response &res)
{
    static u_int32_t requestId = 100;
    requestId++;
    mavlink_ld_camera_trigger_t trigger;
    trigger.request_token = requestId;
    trigger.frequency = 0;
    mavlink_message_t message;
    mavlink_msg_ld_camera_trigger_encode(1,0,&message, &trigger);
    client->write_mavlink_msg(message);

    mavlink_ld_camera_strobe_t strobe;

    for (size_t i =0; i<150; i ++)
    {

        client->getCameraStrobe(strobe);
        if (strobe.request_token == requestId)
        {
            res.strobeTime.fromSec(1.0*strobe.time_strob_usec/1E6);
            res.triggerTime.fromSec(1.0*strobe.time_trigg_usec/1E6);

            return true;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(2));
    }
    return false;
}

void callbackHearbeat(const ros::TimerEvent&)
{

	mavlink_heartbeat_t heart_beat;
	heart_beat.type = MAV_TYPE_GENERIC;
	heart_beat.autopilot = MAV_AUTOPILOT_GENERIC;
	heart_beat.base_mode = 	MAV_MODE_MANUAL_ARMED;
	heart_beat.system_status = MAV_STATE_ACTIVE;
	heart_beat.mavlink_version = 3;
	mavlink_message_t message;
	mavlink_msg_heartbeat_encode(1,0,&message, &heart_beat);

	client->write_mavlink_msg(message);

	mavlink_set_rotation_velocity_t rpm_rqst;

}

void callbackAng(const ros::TimerEvent&)
{

	 mandala_unit_driver::encoder_stamped e;
	 e.angle = client->ang+ offset;
	 e.encoder = 0;
	 uint64_t t = (uint64_t)3600 * (uint64_t)1000 * (uint64_t)1000;
	 e.timestamp = client->timestamp % t ;
	 angPub.publish(e);
	 double hours = 3600 * ( ((int)ros::Time::now().toSec()) / 3600);
	 ros::Time pps_ts = ros::Time(hours + 0.001*e.timestamp);
   std_msgs::UInt64 d;
   d.data =  client->timestamp;
   timeStampPub.publish(d);

}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "m3d_driver");
    ros::NodeHandle node ("~");
    //tf::TransformBroadcaster m3dRot;
    angPub = node.advertise<mandala_unit_driver::encoder_stamped>("current_angle", 100);
    timeStampPub = node.advertise<std_msgs::UInt64>("current_timestamp", 100);
    ros::Subscriber s1 =node.subscribe("velocity", 1, velocityCallback);
    ros::ServiceServer service = node.advertiseService("triggerLadyBug", triggerCamera);

    std::string transportProtocol = "UDP";
    std::string udp_hostIP = "0.0.0.0";
    std::cout << udp_hostIP << "\n";
    boost::asio::io_service io_service;
    client =   dynamic_cast<mavlink_client*>(new mavlink_client_udp(io_service, udp_hostIP, "14550"));
    boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

    int i = 0;
    //tf::TransformBroadcaster* m3dRot;
    //m3dRot = new tf::TransformBroadcaster();
    double offset_deg =0;
    node.param<double>("offset_deg", offset_deg, 0);
    offset =  (1.0 * offset_deg /180) * M_PI;

    ros::Timer timer1 = node.createTimer(ros::Duration(1.00), callbackHearbeat);
    ros::Timer timer2 = node.createTimer(ros::Duration(0.01), callbackAng);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();



}
