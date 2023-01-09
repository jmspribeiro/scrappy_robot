cmake_minimum_required(VERSION 3.0.2)
project(scrappy_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/setup_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)