# uav_utils ROS Package

Provide some widely used utilities in uav development. **No library. Only headers.**

Dependency: Eigen

This package is deliberately designed not to find Eigen automatically, because there are many cases that you would like to use your own Eigen instead of that in the system directory (**/usr/include/eigen3**).

### How to resolve "fatal error: Eigen/Dense: No such file or directory" problem

1. The most simple and naive solution:

	Modify `your-ros-package/CMakeList.txt` like this:
	
	```cmake

	include_directories(
		...
		...
		/usr/include/eigen3
		...
	)
	```

2. Use **FindEigen.cmake** provided by ROS
	
	For details, see [https://github.com/ros/cmake_modules/blob/0.3-devel/README.md](https://github.com/ros/cmake_modules/blob/0.3-devel/README.md)

3. Specify your own Eigen
	
	* For recent releases of Eigen, cmake support is integrated. You may modify your `CMakeLists.txt` like this:

	``` cmake
	find_package(Eigen3 REQUIRED NO_DEFAULT_PATH PATHS /opt/eigen3.3/lib/x86_64-linux-gnu/cmake)
	include_directories(
  		...
  		${EIGEN3_INCLUDE_DIR}
  		${catkin_INCLUDE_DIRS}
  		...
	)

	```

	* For other third-party **FindEigen.cmake**, you have to read the cmake file to find correct **library-name** (Eigen, EIGEN, Eigen3, EIGEN3, etc.) and **include-path** (*_INCLUDE_DIRECTORIES, *_INCLUDE_DIRECTORY, *_INCLUDE_DIR, *_INCLUDE_DIRECTORIES, etc.)
