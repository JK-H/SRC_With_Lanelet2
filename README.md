# Full Coverage With Lanelet2

## Overview

Plan and visualize the full coverage path in a Lanelet2 Map.

Based on
* https://github.com/AbangLZU/ad_with_lanelet2.git


## Installation


### Manual installation

In case you want to build it in your own way (without the above Docker image) use these instructions.

Lanelet2 uses [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html) for building and is targeted towards Linux.

At least C++14 is required.

### Dependencies
Besides [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html), the dependencies are
* `Boost` (from 1.58)
* `eigen3`
* [`mrt_cmake_modules`](https://github.com/KIT-MRT/mrt_cmake_modules), a CMake helper library
* `pugixml` (for lanelet2_io)
* `boost-python, python2 or python3` (for lanelet2_python)
* `geographiclib` (for lanelet2_projection)
* `rosbash` (for lanelet2_examples)

For Ubuntu, the steps are the following:
* [Set up ROS](http://wiki.ros.org/ROS/Installation), and install at least `rospack`, `catkin` and `mrt_cmake_modules` (e.g. `ros-melodic-rospack`, `ros-melodic-catkin`, `ros-melodic-mrt-cmake-modules`):
```
sudo apt-get install ros-melodic-rospack ros-melodic-catkin ros-melodic-mrt-cmake-modules ros-melodic-unique-id

FOR ROS-kinetic:
sudo apt-get install ros-kinetic-unique-id
```

* Install the dependencies above:
```bash
sudo apt-get install libboost-dev libeigen3-dev libgeographic-dev libpugixml-dev libpython-dev libboost-python-dev python-catkin-tools
```

**On 16.04 and below**, `mrt_cmake_modules` is not available in ROS and you have to clone it into your workspace (`git clone https://github.com/KIT-MRT/mrt_cmake_modules.git`).
```shell
cd catkin_ws/src
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
```

### Building
As usual with Catkin, after you have sourced the ros installation, you have to create a workspace and clone all required packages there. Then you can build.
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir catkin_ws && cd catkin_ws && mkdir src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo # build in release mode (or whatever you prefer)
cd src
git clone https://gitlab.senseauto.com/huangjikai/full_coverage_with_lanelet2.git
cd ..
catkin build
```

Note:
build error:
```
error: unable to find numeric literal operator ‘operator""Q’
BOOST_DEFINE_MATH_CONSTANT...
```
need to modify the *CMakeLists.txt* files in following path:
```
./map/lanelet2_extension
./planning/mission_planning/mission_planner
./map/util/lanelet2_map_preprocessor
```

```c
#add_compile_options(-std=c++14)
add_compile_options(-std=c++11) 
#add_compile_options(-std=gnu++11)
add_compile_options(-fext-numeric-literals)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

```

## Running
```shell
cd your_ros_workspace
source devel/setup.bash
roslaunch src/ad_with_lanelet2/run_map_simulator.launch
```
In Rviz:
Set start point by 2D Pose Estimate.
Set goal point by 2D Nav Goal.
The full_coverage_path is generated and displayed dynamically.
