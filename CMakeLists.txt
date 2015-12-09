cmake_minimum_required(VERSION 2.8.3)
project(heavy_duty_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs image_transport cv_bridge)
find_package(OpenCV REQUIRED)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare catkin package
catkin_package()

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(fuse_depth src/fuse_depth.cpp)
add_executable(construct_wrenches src/construct_wrenches.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fuse_depth ${catkin_LIBRARIES})
target_link_libraries(fuse_depth ${OpenCV_LIBS})
target_link_libraries(construct_wrenches ${OpenCV_LIBS})
target_link_libraries(construct_wrenches ${catkin_LIBRARIES})
