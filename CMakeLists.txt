cmake_minimum_required(VERSION 2.8.3)
project(auto_takeoff)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		tf
		rotors_model
		rotors_control
		mav_msgs
		rosbag
		octomap_msgs
		planning_msgs
		geometry_msgs
		cmake_modules
		glog_catkin
		image_transport
		cv_bridge
        std_msgs
        message_generation
        gazebo
		)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

## add message
add_message_files(
        FILES
        WindGust.msg
)
## Generate added messages and services
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

## Declare a catkin package
catkin_package(
		CATKIN_DEPENDS
        message_runtime
		roscpp
		cv_bridge
		mavros
		mav_msgs
		image_transport
		INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
		LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
		CATKIN_DEPENDS roscpp tf rotors_model mav_msgs octomap_ros octomap_msgs planning_msgs geometry_msgs glog_catkin
		DEPENDS eigen gazebo octomap opencv glog_catkin
)





add_executable(auto_takeoff src/auto_takeoff_and_loiter.cpp)
target_link_libraries(auto_takeoff ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )
add_dependencies(auto_takeoff auto_takeoff_generate_messages_cpp)

add_library(my_wind_plugin src/gazebo_wind_plugin.cpp)
target_link_libraries(my_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(my_wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(test_plugin src/test_plugin.cpp)
target_link_libraries(test_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(test_plugin ${catkin_EXPORTED_TARGETS})

add_executable(wind_publisher src/wind_broadcaster.cpp)
target_link_libraries(wind_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )
add_dependencies(wind_publisher auto_takeoff_generate_messages_cpp)
