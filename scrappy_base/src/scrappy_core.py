#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from bts7960 import rpi_JGB37545

def twistCb(msg):
    pass

def scrappyCore():
    rospy.init_node('scrappy_core', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, twistCb)
    rospy.spin()

if __name__ == '__main__':
    scrappyCore()
