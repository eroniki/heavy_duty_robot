cmake_minimum_required(VERSION 2.8.3)
project(heavy_duty_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs image_transport cv_bridge)
find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare catkin package
catkin_package(
    #INCLUDE_DIRS include/wrench_detection
  LIBRARIES wrench_detection
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib  
)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  ${/usr/include}
  ${/usr/lib}
  ${/opt/ros/indigo/lib}
  ${ARMADILLO_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(fuse_depth src/fuse_depth.cpp)
add_executable(construct_wrenches src/construct_wrenches.cpp)
add_executable(wrench_detection_node src/wrench_detection_node.cpp)

add_dependencies(wrench_detection_node 
    ${catkin_EXPORTED_TARGETS}
)


## Specify libraries to link a library or executable target against
target_link_libraries(fuse_depth ${catkin_LIBRARIES})
target_link_libraries(fuse_depth ${OpenCV_LIBS})
target_link_libraries(construct_wrenches ${OpenCV_LIBS})
target_link_libraries(construct_wrenches ${catkin_LIBRARIES})
target_link_libraries(wrench_detection_node 
    ${catkin_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
)
