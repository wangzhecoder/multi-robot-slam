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

set_target_properties(multi_robot_slam_node
                      PROPERTIES OUTPUT_NAME multi_slam
                      PREFIX "")


add_executable(imageConverter src/image_converter.cpp)