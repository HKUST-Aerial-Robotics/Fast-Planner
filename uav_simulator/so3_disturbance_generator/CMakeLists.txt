cmake_minimum_required(VERSION 2.8.3)
project(so3_disturbance_generator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf
  pose_utils
  dynamic_reconfigure
)

catkin_package()

###########
## Build ##
###########
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(so3_disturbance_generator src/so3_disturbance_generator.cpp)
add_dependencies(so3_disturbance_generator ${PROJECT_NAME}_gencfg)

target_link_libraries(so3_disturbance_generator 
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   pose_utils 
)

