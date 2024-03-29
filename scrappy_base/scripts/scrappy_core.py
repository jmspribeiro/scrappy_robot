#!/usr/bin/env python
import sys
import rospy
from scrappy_base.speed_controller import speedController

def main(args):
  rospy.init_node('scrappy_core', anonymous=True)

  # open loop commands for now
  innerloops = speedController()

  # call destructor on shutdown
  rospy.on_shutdown(innerloops.__del__)
  rospy.spin()

  return 0
  
if __name__ == '__main__':
    sys.exit(main(sys.argv))

    
