cmake_minimum_required(VERSION 2.8.3)
project(tuw_keyboard2twist)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure 
  tuw_geometry
)



find_package(OpenCV REQUIRED)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tuw_keyboard2twist tuw_keyboard2twist_node
# CATKIN_DEPENDS tuw_gazebo_plugins
  DEPENDS OpenCV
)

include_directories(
  ./include/
  ${catkin_INCLUDE_DIRS}
)

add_library          (tuw_keyboard2twist src/tuw_keyboard2twist.cpp )
add_dependencies     (tuw_keyboard2twist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tuw_keyboard2twist ${OpenCV_LIBS} ${CURSES_LIBRARY} )

add_executable       (tuw_keyboard2twist_node src/tuw_keyboard2twist_node.cpp)
add_dependencies     (tuw_keyboard2twist_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(tuw_keyboard2twist_node tuw_keyboard2twist ${catkin_LIBRARIES} ${CURSES_LIBRARY} )
