cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_slam)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
)
find_package(OpenCV 3 REQUIRED)
message("OpenCV_INCLUDE_DIRS:"${OpenCV_INCLUDE_DIRS}\n)
message("OpenCV_LIBRARIES:"${OpenCV_LIBRARIES}\n)
# ${OpenCV_INCLUDE_DIRS}
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#link_directories(/usr/lib/x86_64-linux-gnu)
message("catkin_INCLUDE_DIRS":${catkin_INCLUDE_DIRS}\n)
catkin_package()

add_executable(imageConverter src/imageConverter.cpp)
#${OpenCV_LIBRARIES}
target_link_libraries(imageConverter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
message("target_des:"${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS imageConverter
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
