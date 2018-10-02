find_package(OpenCV REQUIRED)
include_directories(
include
${OpenCV_INCLUDE_DIRS}
)



add_library (controller_lib
math_helper.cpp
string_convertor.cpp
colormod.cpp
)

target_link_libraries(controller_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
