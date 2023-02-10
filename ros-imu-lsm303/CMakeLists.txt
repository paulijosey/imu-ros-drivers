cmake_minimum_required(VERSION 2.8.3)
project(ros-imu-lsm303)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  genmsg
  computer_vision_msgs
  opt_flow_msgs
  camera_info_manager
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

# generate_messages(DEPENDENCIES std_msgs)
#roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS kdl_parser roscpp rospy std_msgs
#  DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( 
	include 
	src/IMU 
	src/sys 
	src/CAM
	${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(imu		 	src/IMU/Compass.cpp
				src/IMU/DCM.cpp
				src/IMU/IMU.cpp
				src/IMU/L3G.cpp
				src/IMU/LSM303.cpp
				src/IMU/Matrix.cpp
				src/IMU/Vector.cpp)
add_library(cam 
			src/CAM/config.cpp 
			src/CAM/v4l2_camera.cpp 
			src/CAM/v4l2_camera_device.cpp)

target_link_libraries(imu cam ${catkin_LIBRARIES})

## Declare a cpp executable
#add_executable(imu_control src/imu_control.cpp src/sys/Timer.cpp src/sys/I2C.cpp)
#target_link_libraries(imu_control imu ${catkin_LIBRARIES})
add_executable(imu_node src/imu_node.cpp src/sys/Timer.cpp src/sys/I2C.cpp)
target_link_libraries(imu_node imu cam ${catkin_LIBRARIES})
