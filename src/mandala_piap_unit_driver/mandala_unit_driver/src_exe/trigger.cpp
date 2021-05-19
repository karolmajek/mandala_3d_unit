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
#include <mandala_unit_develop_driver/encoder_stamped.h>
#include <time.h>
#include <mandala_unit_msgs/cameraTrigger.h>
#include "roscpp/Logger.h"
mavlink_client * client = NULL;


ros::Publisher angPub;
int speed = 0;
void velocityCallback(const std_msgs::Float32ConstPtr& msg)
{
    speed = msg->data;
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
	rpm_rqst.rpm = speed;
	//std::cout << speed << "\n";
	mavlink_msg_set_rotation_velocity_encode(1,0,&message, &rpm_rqst);
	client->write_mavlink_msg(message);
}

void callbackAng(const ros::TimerEvent&)
{

	 //std::cout << "aaaa : " << client->ang << "\n";
	 //q.setRPY(0,-M_PI_2, 2*M_PI*(1.0*client->ang/(7600*2)) );

	 mandala_unit_develop_driver::encoder_stamped e;
	 e.angle = client->ang;
	 e.encoder = 0;
	 uint64_t t = (uint64_t)3600 * (uint64_t)1000 * (uint64_t)1000;
	 e.timestamp = client->timestamp % t ;
	 angPub.publish(e);
	 double hours = 3600 * ( ((int)ros::Time::now().toSec()) / 3600);
	 ros::Time pps_ts = ros::Time(hours + 0.001*e.timestamp);


}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "m3d_driver");
    ros::NodeHandle node ("~");
    //tf::TransformBroadcaster m3dRot;

    std::string transportProtocol = "UDP";
    std::string udp_hostIP = "0.0.0.0";
    std::cout << udp_hostIP << "\n";
    boost::asio::io_service io_service;
    client =   dynamic_cast<mavlink_client*>(new mavlink_client_udp(io_service, udp_hostIP, "14550"));
    boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));


    while(ros::ok)
    {
    ros::spinOnce();
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
              std::cout << 1.0*strobe.time_strob_usec/1E6 << "\n";
              break;
          }
          boost::this_thread::sleep(boost::posix_time::milliseconds(2));
      }
    }



}
