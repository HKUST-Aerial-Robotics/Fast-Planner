# Fast-Planner

**Fast-Planner** is developed aiming to enable quadrotor fast flight in complex unknown environments. It contains a rich set of carefully designed planning algorithms. It also provides a foundational code framework and algorithms that support several popular open-source drone projects, including [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner),
[FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) and [RACER](https://github.com/SYSU-STAR/RACER), etc.

**News**: 

- __Mar 13, 2021__: Code for fast autonomous exploration is available now! Check this [repo](https://github.com/HKUST-Aerial-Robotics/FUEL) for more details.

- __Oct 20, 2020__: Fast-Planner is extended and applied to fast autonomous exploration. Check this [repo](https://github.com/HKUST-Aerial-Robotics/FUEL) for more details.

__Authors__: [Boyu Zhou](http://sysu-star.com) and [Shaojie Shen](http://uav.ust.hk/group/) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/), [Fei Gao](http://zju-fast.com/fei-gao/) from ZJU FAST Lab.
<!-- - __B-spline trajectory optimization guided by topological paths__:
<p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/TopoTraj/blob/master/files/icra20_1.gif" width = "420" height = "237"/>
  <img src="https://github.com/HKUST-Aerial-Robotics/TopoTraj/blob/master/files/icra20_2.gif" width = "420" height = "237"/>
</p> -->

<p align="center">
  <img src="files/raptor1.gif" width = "400" height = "225"/>
  <img src="files/raptor2.gif" width = "400" height = "225"/>
  <img src="files/icra20_2.gif" width = "400" height = "225"/>
  <img src="files/ral19_2.gif" width = "400" height = "225"/>
  <!-- <img src="files/icra20_1.gif" width = "320" height = "180"/> -->
</p>

Complete videos: 
[video1](https://www.youtube.com/watch?v=NvR8Lq2pmPg&feature=emb_logo),
[video2](https://www.youtube.com/watch?v=YcEaFTjs-a0), 
[video3](https://www.youtube.com/watch?v=toGhoGYyoAY). 
Demonstrations about this work have been reported on the IEEE Spectrum: [page1](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-nasa-lemur-robot), [page2](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-india-space-humanoid-robot),
[page3](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-soft-exoskeleton-glove-extra-thumb) (search for _HKUST_ in the pages).

To run this project in minutes, check [Quick Start](#1-Quick-Start). Check other sections for more detailed information.

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.






## Table of Contents

* [Quick Start](#1-Quick-Start)
* [Algorithms and Papers](#2-Algorithms-and-Papers)
* [Setup and Config](#3-Setup-and-Config)
* [Run Simulations](#4-run-simulations)
* [Use in Your Application](#5-use-in-your-application)
* [Updates](#6-updates)
* [Known issues](#known-issues)


## 1. Quick Start

This project has been tested on Ubuntu 18.04(ROS Melodic) and 20.04(ROS Noetic).

Firstly, you should install __nlopt v2.7.1__:
```
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

Next, you can run the following commands to install other required tools:
```
sudo apt-get install libarmadillo-dev
```

Then simply clone and compile our package (using ssh here):

```
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git
cd ../ 
catkin_make
```

You may check the detailed [instruction](#3-setup-and-config) to setup the project. 
After compilation you can start the visualization by: 

```
source devel/setup.bash && roslaunch plan_manage rviz.launch
```
and start a simulation (run in a new terminals): 
```
source devel/setup.bash && roslaunch plan_manage kino_replan.launch
```
You will find the random map and the drone in ```Rviz```. You can select goals for the drone to reach using the ```2D Nav Goal``` tool. A sample simulation is showed [here](#demo1).


## 2. Algorithms and Papers

The project contains a collection of robust and computationally efficient algorithms for quadrotor fast flight:
* Kinodynamic path searching
* B-spline-based trajectory optimization
* Topological path searching and path-guided optimization
* Perception-aware planning strategy (to appear)

These methods are detailed in our papers listed below. 

Please cite at least one of our papers if you use this project in your research: [Bibtex](files/bib.txt).

- [__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.
- [__Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths__](https://arxiv.org/abs/1912.12644), Boyu Zhou, Fei Gao, Jie Pan and Shaojie Shen, IEEE International Conference on Robotics and Automation (__ICRA__), 2020.
- [__RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight__](https://arxiv.org/abs/2007.03465), Boyu Zhou, Jie Pan, Fei Gao and Shaojie Shen, IEEE Transactions on Robotics (__T-RO__). 


All planning algorithms along with other key modules, such as mapping, are implemented in __fast_planner__:

- __plan_env__: The online mapping algorithms. It takes in depth image (or point cloud) and camera pose (odometry) pairs as input, do raycasting to update a probabilistic volumetric map, and build an Euclidean signed distance filed (ESDF) for the planning system. 
- __path_searching__: Front-end path searching algorithms. 
  Currently it includes a kinodynamic path searching that respects the dynamics of quadrotors.
  It also contains a sampling-based topological path searching algorithm to generate multiple topologically distinctive paths that capture the structure of the 3D environments. 
- __bspline__: A implementation of the B-spline-based trajectory representation.
- __bspline_opt__: The gradient-based trajectory optimization using B-spline trajectory.
- __active_perception__: Perception-aware planning strategy, which enable to quadrotor to actively observe and avoid unknown obstacles, to appear in the future.
- __plan_manage__: High-level modules that schedule and call the mapping and planning algorithms. Interfaces for launching the whole system, as well as the configuration files are contained here.

Besides the folder __fast_planner__, a lightweight __uav_simulator__ is used for testing.


## 3. Setup and Config

### Prerequisites

1. Our software is developed and tested in Ubuntu 18.04(ROS Melodic) and 20.04(ROS Noetic). 
   
2. We use [**NLopt**](https://nlopt.readthedocs.io/en/latest/NLopt_Installation) to solve the non-linear optimization problem. The __uav_simulator__ depends on the C++ linear algebra library __Armadillo__. The two dependencies can be installed by the following command.

Firstly, you should install __nlopt v2.7.1__:
```
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

Next, you can run the following commands to install other required tools:
```
sudo apt-get install libarmadillo-dev
```

### Build on ROS

After the prerequisites are satisfied, you can clone this repository to your catkin workspace and catkin_make. A new workspace is recommended:

```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git
  cd ../
  catkin_make
```

If you encounter problems in this step, please first refer to existing __issues__, __pull requests__ and __Google__ before raising a new issue.

Now you are ready to [run a simulation](#4-run-simulations).

### Use GPU Depth Rendering (can be skipped optionally)

This step is not mandatory for running the simulations. However, if you want to run the more realistic depth camera in __uav_simulator__, installation of [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) is needed. Otherwise, a less realistic depth sensor model will be used.

 The **local_sensing** package in __uav_simulator__ has the option of using GPU or CPU to render the depth sensor measurement. By default, it is set to CPU version in CMakeLists:
 
 ```
 set(ENABLE_CUDA false)
 # set(ENABLE_CUDA true)
 ```
However, we STRONGLY recommend the GPU version, because it generates depth images more like a real depth camera.
To enable the GPU depth rendering, set ENABLE_CUDA to true, and also remember to change the 'arch' and 'code' flags according to your graphics card devices. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility).

```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 
``` 
For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 4. Run Simulations

Run [Rviz](http://wiki.ros.org/rviz) with our configuration firstly:

```
  <!-- go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage rviz.launch
```

Then run the quadrotor simulator and __Fast-Planner__. 
Several examples are provided below:

### Kinodynamic Path Searching & B-spline Optimization

In this method, a kinodynamic path searching finds a safe, dynamically feasible, and minimum-time initial trajectory in the discretized control space. 
Then the smoothness and clearance of the trajectory are improved by a B-spline optimization.
To test this method, run:

```
  <!-- open a new terminal, go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage kino_replan.launch
```

Normally, you will find the randomly generated map and the drone model in ```Rviz```. At this time, you can trigger the planner using the ```2D Nav Goal``` tool. When a point is clicked in ```Rviz```, a new trajectory will be generated immediately and executed by the drone. A sample is displayed below:

<!-- add some gif here -->
 <p id="demo1" align="center">
  <img src="files/ral19_3.gif" width = "480" height = "270"/>
 </p>

Related algorithms are detailed in [this paper](https://ieeexplore.ieee.org/document/8758904).



### Topological Path Searching & Path-guided Optimization

This method features searching for multiple trajectories in distinctive topological classes. Thanks to the strategy, the solution space is explored more thoroughly, avoiding local minima and yielding better solutions.
Similarly, run:

```
  <!-- open a new terminal, go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage topo_replan.launch
```

then you will find the random map generated and can use the ```2D Nav Goal``` to trigger the planner:

<!-- add some gif here -->
 <p align="center">
  <img src="files/icra20_3.gif" width = "480" height = "270"/>
 </p>

Related algorithms are detailed in [this paper](https://arxiv.org/abs/1912.12644).


### Perception-aware Replanning

The code will be released after the publication of [associated paper](https://arxiv.org/abs/2007.03465).


## 5. Use in Your Application

If you have successfully run the simulation and want to use __Fast-Planner__ in your project,
please explore the files kino_replan.launch or topo_replan.launch.
Important parameters that may be changed in your usage are contained and documented.

Note that in our configuration, the size of depth image is 640x480. 
For higher map fusion efficiency we do downsampling (in kino_algorithm.xml, skip_pixel = 2).
If you use depth images with lower resolution (like 256x144), you might disable the downsampling by setting skip_pixel = 1. Also, the _depth_scaling_factor_ is set to 1000, which may need to be changed according to your device.

Finally, for setup problem, like compilation error caused by different versions of ROS/Eigen, please first refer to existing __issues__, __pull request__, and __Google__ before raising a new issue. Insignificant issue will receive no reply.


## 6. Updates

- __Oct 20, 2020__: Fast-Planner is extended and applied to fast autonomous exploration. Check this [repo](https://github.com/HKUST-Aerial-Robotics/FUEL) for more details.
  
- __July 5, 2020__: We will release the implementation of paper: _RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight_ (submitted to TRO, under review) in the future.

- __April 12, 2020__: The implementation of the ICRA2020 paper: _Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths_ is available.

- __Jan 30, 2020__: The volumetric mapping is integrated with our planner. It takes in depth image and camera pose pairs as input, do raycasting to fuse the measurements, and build a Euclidean signed distance field (ESDF) for the planning module.


## Acknowledgements
  We use **NLopt** for non-linear optimization.

## Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.


## Disclaimer
This is research code, it is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of merchantability or fitness for a particular purpose.
