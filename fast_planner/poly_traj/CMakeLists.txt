cmake_minimum_required(VERSION 2.8.3)
project(poly_traj)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poly_traj
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_library( poly_traj 
    src/polynomial_traj.cpp 
    )
target_link_libraries( poly_traj
    ${catkin_LIBRARIES} 
    )  
