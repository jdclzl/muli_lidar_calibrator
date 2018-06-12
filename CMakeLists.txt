cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_calibrator)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        )

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        geometry_msgs
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar Calibrator
add_library(multi_lidar_calibrator_lib SHARED
        src/multi_lidar_calibrator.cpp
        include/multi_lidar_calibrator.h)
add_library(joint_two_pointclouds_lib SHARED
        src/joint_two_pointclouds.cpp
        include/joint_two_pointclouds.h)

target_include_directories(multi_lidar_calibrator_lib PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )
target_include_directories(joint_two_pointclouds_lib PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(multi_lidar_calibrator_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_executable(multi_lidar_calibrator
        src/multi_lidar_calibrator_node.cpp 
        )
add_executable(joint_two_pointclouds
        src/joint_two_pointclouds_node.cpp 
        )
target_link_libraries(joint_two_pointclouds_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

target_include_directories(joint_two_pointclouds PRIVATE
        include)
target_include_directories(multi_lidar_calibrator PRIVATE
        include)

target_link_libraries(multi_lidar_calibrator
        multi_lidar_calibrator_lib)
target_link_libraries(joint_two_pointclouds
        joint_two_pointclouds_lib)
