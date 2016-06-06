cmake_minimum_required(VERSION 2.8.3)
project(tuw_gui2iws)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(tuw_control REQUIRED)
if(tuw_control_FOUND)
  MESSAGE(STATUS "Found package tuw_control, compiling also gui2iws_node !")
  
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
		tuw_gazebo_plugins
		tuw_control
	)
  
	find_package(OpenCV REQUIRED)

	set(CURSES_NEED_NCURSES TRUE)
	find_package(Curses REQUIRED)
	message(${CURSES_LIBRARY})

	generate_dynamic_reconfigure_options(
    cfg/tuw_gui2iws.cfg
    )
    
    catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES tuw_gui2iws_node
	  CATKIN_DEPENDS tuw_gazebo_plugins tuw_control
	  DEPENDS OpenCV
	)

	include_directories(
	  ./include/
	  ${catkin_INCLUDE_DIRS}
	)

    add_library          (tuw_gui2iws src/tuw_gui2iws.cpp )
    add_dependencies     (tuw_gui2iws ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(tuw_gui2iws ${OpenCV_LIBS} )

    add_executable       (tuw_gui2iws_node src/tuw_gui2iws_node.cpp)
    add_dependencies     (tuw_gui2iws_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg )
    target_link_libraries(tuw_gui2iws_node tuw_gui2iws ${catkin_LIBRARIES} )
  
  
endif()

