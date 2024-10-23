PROJECT(local_sensing_node)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 14)

set(ENABLE_CUDA false)
# set(ENABLE_CUDA true)

if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
  set(CUDA_NVCC_FLAGS 
#       -gencode arch=compute_20,code=sm_20;
#       -gencode arch=compute_20,code=sm_21;
#       -gencode arch=compute_30,code=sm_30;
#       -gencode arch=compute_35,code=sm_35;
#       -gencode arch=compute_50,code=sm_50;
#       -gencode arch=compute_52,code=sm_52; 
#       -gencode arch=compute_60,code=sm_60;
      -gencode arch=compute_61,code=sm_61;
  ) 

  SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem)

  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs dynamic_reconfigure)
  generate_dynamic_reconfigure_options(
    cfg/local_sensing_node.cfg
  )
  catkin_package(
      DEPENDS OpenCV Eigen Boost
      CATKIN_DEPENDS roscpp roslib image_transport pcl_ros
  #INCLUDE_DIRS include
      LIBRARIES depth_render_cuda
  )

  include_directories(
    SYSTEM 
    #include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  CUDA_ADD_LIBRARY( depth_render_cuda
      src/depth_render.cu
  )

  add_executable(
    pcl_render_node
    src/pcl_render_node.cpp
  )
  target_link_libraries( pcl_render_node
    depth_render_cuda
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
else(ENABLE_CUDA)
  find_package(Eigen3 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs)
  
  catkin_package(
      DEPENDS Eigen
      CATKIN_DEPENDS roscpp roslib pcl_ros
  )

  include_directories(
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
  )

  add_executable(
    pcl_render_node
    src/pointcloud_render_node.cpp
  )

  target_link_libraries( pcl_render_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
endif(ENABLE_CUDA)
