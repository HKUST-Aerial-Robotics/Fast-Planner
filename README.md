# Fast-Planner

## News

This package is under active maintenance. New features will be listed here.

- The heading (yaw angle) planner which enables smoother change of heading direction is available.

- The online mapping algorithm is now available. It can take in depth image and camera pose pairs as input, do raycasting to update a probabilistic volumetric map, and build a Euclidean signed distance field (ESDF) for the planning system.

## Overview

__Fast-Planner__ is a robust and efficient planning system that enables agile and fast autonomous flight for quadrotors.
It takes in information from odometry, sensor streams (such as depth images and point cloud), and outputs high-quality trajectories within a few milliseconds.
It can support aggressive and fully autonomous flight even in unknown and cluttered environments.
Demonstrations about the planner have been reported on the [IEEE Spectrum](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-nasa-lemur-robot).




__Authors__: [Boyu Zhou](http://boyuzhou.net), [Fei Gao](https://ustfei.com/) and [Shaojie Shen](http://uav.ust.hk/group/) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/).

__Video__:

<!-- add some gif of the paper video: -->
<p align="center">
  <img src="files/exp1.gif" width = "420" height = "237"/>
<!-- </p> -->

<!-- <p align="center"> -->
  <img src="files/exp2.gif" width = "420" height = "237"/>
</p>

<p align="center">
  <a href="https://youtu.be/XxBw2nmL8t0" target="_blank"><img src="files/title.png" alt="video" width="480" height="270" border="1" /></a>
</p>

This package contains the implementation of __Fast-Planner__ (in folder __fast_planner__) and a lightweight
quadrotor simulator (in __uav_simulator__). Key components are:

- __plan_env__: The online mapping algorithms. It takes in depth image (or point cloud) and camera pose (odometry) pairs as input, do raycasting to update a probabilistic volumetric map, and build an Euclidean signed distance filed (ESDF) for the planning system. 
- __path_searching__: Front-end path searching algorithms. Currently it includes a kinodynamic version of A* algorithm that respects the dynamics of quadrotors. The standard A* is also available. 
- __bspline_opt__: The gradient-based trajectory optimization based on B-spline trajectory representation.
- __plan_manage__: High-level modules that schedule and call the mapping and planning algorithms. Interfaces for launching the whole system, as well as the configuration files are contained here.

If you use __Fast-Planner__ for your application or research, please cite our related paper:

- [__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (RA-L), 2019.
```
@article{zhou2019robust,
  title={Robust and efficient quadrotor trajectory generation for fast autonomous flight},
  author={Zhou, Boyu and Gao, Fei and Wang, Luqi and Liu, Chuhao and Shen, Shaojie},
  journal={IEEE Robotics and Automation Letters},
  volume={4},
  number={4},
  pages={3529--3536},
  year={2019},
  publisher={IEEE}
}
```


## 1. Prerequisites

- Our software is developed and tested in Ubuntu 16.04, [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Other version may require minor modification.

- We use [**NLopt**](https://nlopt.readthedocs.io/en/latest/NLopt_Installation) to solve the non-linear optimization problem.

- The __uav_simulator__ depends on the C++ linear algebra library __Armadillo__, which can be installed by ``` sudo apt-get install libarmadillo-dev ```.

- _Optional_: If you want to run the more realistic depth camera in __uav_simulator__, installation of [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) is needed. Otherwise, a less realistic depth sensor model will be used (See section _Use GPU Depth Rendering_ below).

## 2. Build on ROS

After the prerequisites are satisfied, you can clone this repository to your catkin workspace and catkin_make. A new workspace is recommended:
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git
  cd ../
  catkin_make
```

### Use GPU Depth Rendering (Optional)

 The **local_sensing** package in __uav_simulator__ has the option of using GPU or CPU to render the depth sensor measurement. By default, it is set to CPU version in CMakeLists:
 
 ```
 set(ENABLE_CUDA false)
 # set(ENABLE_CUDA true)
 ```
The GPU version is recommended, because it generates depth images more like a real depth camera.
If you want to use the GPU depth rendering, set ENABLE_CUDA to true, and also remember to change the 'arch' and 'code' flags according to your graphics card devices. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility).

```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 
``` 
For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 3. Run the Simulation

Run [Rviz](http://wiki.ros.org/rviz) with our configuration firstly:

```
  <!-- go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage rviz.launch
```

Then run the quadrotor simulator and __Fast-Planner__:

```
  <!-- open a new terminal, go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage simulation.launch
```

Normally, you will find the randomly generated map and the drone model in ```Rviz```. At this time, you can select a goal for the drone using the ```2D Nav Goal``` tool. When a goal is set successfully, a new trajectory will be generated immediately and executed by the drone. A sample is displayed below:

<!-- add some gif here -->
 <p align="center">
  <img src="files/exp3.gif" width = "640" height = "360"/>
 </p>

## 4. Use in Your Application

If you have successfully run the simulation and want to use __Fast-Planner__ in your project,
please explore the simulation.launch file.
Important parameters that may be changed in your usage are contained and documented.

Note that in our configuration, the size of depth image is 640x480. 
For higher map fusion efficiency we do downsampling (in kino_algorithm.xml, skip_pixel = 2).
If you use depth images with lower resolution (like 256x144), you might disable the downsampling by setting skip_pixel = 1. Also, the _depth_scaling_factor_ is set to 1000, which may need to be changed according to your device.

Finally, please kindly give a STAR to this repo if it helps your research or work, thanks! :)

## 5. Acknowledgements
  We use **NLopt** for non-linear optimization.

## 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.


## 7. Disclaimer
This is research code, it is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of merchantability or fitness for a particular purpose.
