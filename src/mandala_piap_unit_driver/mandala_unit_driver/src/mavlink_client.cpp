/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   mavlink_client.cpp
 * Author: michal
 *
 * Created on February 8, 2016, 10:53 PM
 */
#include <ros/console.h>
#include "mavlink_client.h"

using boost::asio::ip::udp;


mavlink_client::mavlink_client(
                boost::asio::io_service& io_service
        ) : io_service_(io_service) {
			currentWaypointId = -1;
			currentWaypointCount = -1;
			newPlan = false;
        }
void mavlink_client::getCameraStrobe(mavlink_ld_camera_strobe_t &t)
{
    lockMeasurment.lock();
    t = lastCameraStrobe;
    lockMeasurment.unlock();
}
void mavlink_client::handle_receive_from(char* data, size_t bytes_recvd)
 {
	mavlink_message_t message;
	mavlink_status_t status;
	for (int i = 0; i < bytes_recvd; i++) {

			if (mavlink_parse_char(1, data[i], &message, &status)) {

//				if (message.msgid == MAVLINK_MSG_ID_NAMED_VALUE_INT)
//				{
//					mavlink_named_value_int_t tt;
//					mavlink_msg_named_value_int_decode(&message,&tt);
//					if (strcmp(tt.name, "enc1") == 0)
//					{
////						std::cout << tt.name <<"\t" << tt.value <<"\n";
//                                        ang = tt.value;
//                                        timestamp = tt.time_boot_ms;
//					}
//				}

				if (message.msgid == MAVLINK_MSG_ID_UNIT_ENCODERS)
				{
					mavlink_unit_encoders_t tt;
					mavlink_msg_unit_encoders_decode(&message,&tt);
					lockMeasurment.lock();
					timestamp = tt.ts;
					ang = tt.encoder1;
                    lockMeasurment.unlock();

				}

                if (message.msgid == MAVLINK_MSG_ID_LD_CAMERA_STROBE)
                {
                    mavlink_ld_camera_strobe_t t;
                    mavlink_msg_ld_camera_strobe_decode(&message,&t);
                    std::cout << "recieved strobe with given token :" << t.request_token <<"\t" <<"\n";
                    lockMeasurment.lock();
                    lastCameraStrobe = t;
                    lockMeasurment.unlock();
                }


				//std::cout << "MESSAGE ("<< (int) message.msgid <<") FROM " <<  (int) message.sysid <<"\t comp_id: "<< (int) message.compid << std::endl ;

			}

	}

 }

void mavlink_client::requestWaypointFromBase(int id)
{
	mavlink_mission_request_t item;
	item.target_component=1;
	item.target_system=0;
	item.seq = id;
	mavlink_message_t message;
	mavlink_msg_mission_request_encode(1,0,&message,&item);
	this->write_mavlink_msg(message);
}
