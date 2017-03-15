cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_slam)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    rosconsole
    message_filters
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    visualization_msgs
    message_generation
)
#set(OpenCV_DIR /usr/local/opencv2.4.9/share/OpenCV)
#find_package(OpenCV 2.4.9 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
#find_package(Eigen REQUIRED)
#message("OpenCV_INCLUDE_DIRS:"${OpenCV_INCLUDE_DIRS}\n)
#message("OpenCV_LIBRARIES:"${OpenCV_LIBRARIES}\n)
# ${OpenCV_INCLUDE_DIRS}
#include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
#include_directories(${catkin_INCLUDE_DIRS} /usr/local/opencv2.4.9/include)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
#link_directories(/usr/lib/x86_64-linux-gnu)
message("catkin_INCLUDE_DIRS":${catkin_INCLUDE_DIRS}\n)
add_message_files(
	FILES
	Scenenode.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

add_executable(imageConverter src/imageConverter.cpp)
#${OpenCV_LIBRARIES}
target_link_libraries(imageConverter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(imageConverter ${catkin_LIBRARIES} opencv_videostab opencv_video opencv_superres opencv_stitching opencv_photo opencv_ocl opencv_objdetect opencv_nonfree opencv_ml opencv_legacy opencv_imgproc opencv_highgui opencv_gpu opencv_flann opencv_features2d opencv_core opencv_contrib opencv_calib3d)
message("target_des:"${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS imageConverter
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
