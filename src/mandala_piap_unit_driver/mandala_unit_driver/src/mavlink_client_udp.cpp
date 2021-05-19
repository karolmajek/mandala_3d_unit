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

#include "mavlink_client_udp.h"
#include "roscpp/Logger.h"
#include <ros/ros.h>
using boost::asio::ip::udp;
typedef boost::asio::detail::socket_option::boolean<SOL_SOCKET, SO_REUSEPORT> reuse_port;

mavlink_client_udp::mavlink_client_udp(
				boost::asio::io_service& io_service,
                const std::string& host,
                const std::string& port
        ) : mavlink_client(io_service), socket_(io_service, udp::endpoint(udp::v4(), 14550))
		{

                endpoint_.address(boost::asio::ip::address::from_string("192.168.0.77"));
                endpoint_.port(14450);
								socket_.set_option(reuse_port(true));
                socket_.async_receive_from(
                 boost::asio::buffer(data_, max_length), sender_endpoint_,
                 boost::bind(&mavlink_client_udp::handle_receive_from_udp, this,
                 boost::asio::placeholders::error,
                 boost::asio::placeholders::bytes_transferred));
        }
void mavlink_client_udp::write_mavlink_msg(mavlink_message_t &message)
{
    uint8_t buffer[1024];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    try{
    	socket_.send_to(boost::asio::buffer(buffer, len), endpoint_);
    }
	catch (boost::system::system_error e)
	{
		ROS_WARN_THROTTLE(0.5, e.what());
	}

}
void mavlink_client_udp::handle_receive_from_udp (const boost::system::error_code& error,
        size_t bytes_recvd)
{
	endpoint_ = sender_endpoint_;
	this->handle_receive_from(data_, bytes_recvd);
	socket_.async_receive_from(
	                 boost::asio::buffer(data_, max_length), sender_endpoint_,
	                 boost::bind(&mavlink_client_udp::handle_receive_from_udp, this,
	                 boost::asio::placeholders::error,
	                 boost::asio::placeholders::bytes_transferred));

}
