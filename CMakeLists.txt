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

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(imageConverter src/imageConverter.cpp)
target_link_libraries(imageConverter ${catkin_LIBRARIES})

#install(TARGETS imageConverter
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
