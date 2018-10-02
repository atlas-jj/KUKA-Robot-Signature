cmake_minimum_required(VERSION 2.8.3)
project(kuka_iiwa_ros_controller)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport message_generation)

find_package(OpenCV REQUIRED)
#find_package(VISP REQUIRED)



#add_message_files(
#    FILES
#        targetPoints.msg
#)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#)

catkin_package(
CATKIN_DEPENDS message_runtime
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES signature_drawing
#)
include_directories ("${PROJECT_SOURCE_DIR}/include")
add_subdirectory ("${PROJECT_SOURCE_DIR}/lib")

include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
#include_directories(${VISP_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(controller src/main.cpp src/path_checker.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
#target_link_libraries(controller ${VISP_LIBRARIES})

#target_link_libraries(controller lapack blas)
target_link_libraries (controller controller_lib)
