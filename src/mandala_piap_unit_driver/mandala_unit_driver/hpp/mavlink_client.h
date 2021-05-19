/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   mavlink_client.h
 * Author: michal
 *
 * Created on February 8, 2016, 10:53 PM
 */

#ifndef MAVLINK_CLIENT_H
#define MAVLINK_CLIENT_H
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "mavlink.h"
#include <mutex>
class mavlink_client
{
public:
	mavlink_client(
		boost::asio::io_service& io_service
	);
    void handle_receive_from(char* data, size_t bytes_recvd);
    virtual void write_mavlink_msg(mavlink_message_t &message)=0;
    inline bool isNewPlan () {return newPlan;}
    std::vector<mavlink_mission_item_t> &getPlan()
    {
    	newPlan = false;
    	return  plan;
    }

    bool joystickRecived (mavlink_manual_control_t &msg)
    {
    	if (recivedManual == false) return false;
    	msg = manualControl;
    	recivedManual = false;
    	return true;

    }
    void getCameraStrobe(mavlink_ld_camera_strobe_t &t);
    float ang;
	uint64_t timestamp;
protected:
    boost::asio::io_service& io_service_;
    void requestWaypointFromBase(int id);

private:
    std::mutex lockMeasurment;
    mavlink_ld_camera_strobe_t lastCameraStrobe;
    int currentWaypointId;
    int currentWaypointCount;
    std::vector<mavlink_mission_item_t> plan;
    bool newPlan;

    bool recivedManual;
    mavlink_manual_control_t manualControl;
};
#endif /* MAVLINK_CLIENT_H */
