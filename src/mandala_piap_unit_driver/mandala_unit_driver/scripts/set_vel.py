#!/usr/bin/python
from __future__ import print_function
import rospy
import socket
import time
import sys
from std_msgs.msg import Int32

TCP_PORT = 8888
BUFFER_SIZE = 1024

def write_msg(msg):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.settimeout(1.0)
    # s.send(msg) #Py2
    s.send(msg.encode())
    sys.stdout.flush()
    time.sleep(0.2)
    s.close()
def vel_mode(data):
    msg = "mvel %d \n"%(data.data)
    write_msg(msg)
    rospy.loginfo("setting velocity to %d"%(data.data))


def pos_mode(data):
    speed =45
    msg = "mova %d %d\n"%(speed, data.data)
    write_msg(msg)
    rospy.loginfo("setting pos to %d"%(data.data))

def on_shutdown():
    # turn off motor
    write_msg("moff\n")
if __name__ == '__main__':
    global TCP_IP
    rospy.init_node('unit_motor_control',anonymous=True)
    TCP_IP = rospy.get_param("~unit_ip", '192.168.1.10')
    def_speed = rospy.get_param ("~default_vel", 0)
    rospy.on_shutdown(on_shutdown)
    rospy.loginfo("TCP_IP %s"%(TCP_IP))
    if def_speed != 0:
        t = Int32(def_speed)
        vel_mode(t)
    rospy.Subscriber("vel", Int32, vel_mode)
    rospy.Subscriber("pos", Int32, pos_mode)
    rospy.spin()

