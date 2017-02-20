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

catkin_package(
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        cv_bridge
        std_msgs
        image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imageConverter src/image_converter.cpp)

target_link_libraries(imageConverter,${catkin_LIBRARIES})
set_target_properties(imageConverter
                      PROPERTIES OUTPUT_NAME multi_slam_imageConverter
                      PREFIX "")



