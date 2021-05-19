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

#ifndef MAVLINK_CLIENT_UDP_H
#define MAVLINK_CLIENT_UDP_H
#include <iostream>
#include "mavlink_client.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "mavlink.h"
using boost::asio::ip::udp;
class mavlink_client_udp : public mavlink_client
{
public:
	mavlink_client_udp(
		boost::asio::io_service& io_service, 
		const std::string& host, 
		const std::string& port
	);
    void write_mavlink_msg(mavlink_message_t &message);
    
private:
    udp::socket socket_;
    udp::endpoint endpoint_;
    udp::endpoint sender_endpoint_;
    enum { max_length = 1024 };
    char data_[max_length];

    void handle_receive_from_udp (const boost::system::error_code& error,
            size_t bytes_recvd);
};
#endif /* MAVLINK_CLIENT_H */


