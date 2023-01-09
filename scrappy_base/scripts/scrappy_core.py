#!/usr/bin/env python
import sys
import rospy
from scrappy_base.speed_controller import speedController

def main(args):
  rospy.init_node('scrappy_core', anonymous=True)

  # open loop commands for now
  innerloops = speedController()

  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)

    
