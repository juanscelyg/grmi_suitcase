cmake_minimum_required(VERSION 2.8.3)
project(grmi_suitcase)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  geometry_msgs
  message_generation
  tf
  std_msgs
  nav_msgs
  visualization_msgs
  dynamic_reconfigure)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_install_python(PROGRAMS  scripts/person_detector.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
