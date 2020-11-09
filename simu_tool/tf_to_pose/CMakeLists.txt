cmake_minimum_required(VERSION 2.8.3)
project(tf_to_pose)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        )
# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
        CATKIN_DEPENDS  roscpp rospy std_msgs geometry_msgs tf message_runtime
)
include_directories(
        ${catkin_INCLUDE_DIRS}
)
add_executable(px4_pose px4_pose.cpp)
target_link_libraries(px4_pose ${catkin_LIBRARIES})


