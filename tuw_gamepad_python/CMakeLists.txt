cmake_minimum_required(VERSION 3.0.2)
project(tuw_gamepad_python)

find_package(catkin REQUIRED
  rospy
  roslint
  dynamic_reconfigure
  geometry_msgs
  std_msgs
  tuw_nav_msgs
)

roslint_python()

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/gamepad_dynamic_config.cfg
  cfg/converter/twist_with_orientation_for_iwos_config.cfg
  cfg/converter/joints_iws_for_iwos_config.cfg
  # add converter config here
)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  src/tuw_gamepad_python/gamepad_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
