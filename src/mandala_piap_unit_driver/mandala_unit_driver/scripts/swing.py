#!/usr/bin/python
from __future__ import print_function
import rospy
import socket
import time
import sys
from std_msgs.msg import Int32

TCP_PORT = 8888
BUFFER_SIZE = 1024

def vel_mode(data):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    str = "mvel %d \n"%(data.data)
    rospy.loginfo("setting velocity to %d"%(data.data))
    s.close()

def pos_mode(data):
    speed =45
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    str = "mova %d %d\n"%(speed, data.data)
    rospy.loginfo("setting velocity to %d"%(data.data))
    s.close()
    

if __name__ == '__main__':
    global TCP_IP
    rospy.init_node('unit_motor_control',anonymous=True)
    TCP_IP = rospy.get_param("unit_ip" "192.168.1.10")
    def_speed = get_param ("default_vel", 0)
    if def_speed != 0:
        Int32 t
        t.data = def_speed
        vel_mode(t)
    rospy.Subscriber("vel", Int32, vel_mode)
    rospy.Subscriber("pos", Int32, pos_mode)

    rospy.spin()
