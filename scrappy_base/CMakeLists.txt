cmake_minimum_required(VERSION 3.0.2)
project(scrappy_base)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/scrappy_core.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
