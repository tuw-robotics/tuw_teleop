cmake_minimum_required(VERSION 2.8.3)
project(tuw_keyboard)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure 
)



find_package(OpenCV REQUIRED)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

generate_dynamic_reconfigure_options( 
  cfg/KeyboardControl.cfg 
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tuw_keyboard tuw_keyboard_node
# CATKIN_DEPENDS tuw_gazebo_plugins
  DEPENDS OpenCV
)

include_directories(
  ./include/
  ${catkin_INCLUDE_DIRS}
)

add_library          (${PROJECT_NAME} src/tuw_keyboard.cpp )
add_dependencies     (${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CURSES_LIBRARY} )

add_executable       (tuw_keyboard_node src/tuw_keyboard_node.cpp)
add_dependencies     (tuw_keyboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(tuw_keyboard_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${CURSES_LIBRARY} )
