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
  dynamic_reconfigure
  cv_bridge)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(INCLUDE_DIRS DEPENDS OpenCV)

catkin_install_python(PROGRAMS  scripts/person_detector.py
                                scripts/motors_control.py
                                scripts/visual_control.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
