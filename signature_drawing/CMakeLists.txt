cmake_minimum_required(VERSION 2.8.3)
project(signature_drawing)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport)

find_package(OpenCV REQUIRED)
#find_package(VISP REQUIRED)

catkin_package()

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
add_executable(signature_extractor src/SignatureMain.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(signature_extractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
#target_link_libraries(signature_extractor ${VISP_LIBRARIES})

#target_link_libraries(signature_extractor lapack blas)
target_link_libraries (signature_extractor lib)
