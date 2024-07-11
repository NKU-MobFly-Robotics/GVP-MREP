- [GVP-MREP](#GVP-MREP)
  - [Overview](#1-overview)
  - [Setup](#2-setup)
  - [Parameters](#3-parameters)
  
  <!-- - [Quick Start](#quick-start)
  - [Exploring Different Environments](#exploring-different-environments)
  - [Known issues](#known-issues)
    - [Compilation issue](#compilation-issue)
    - [Unexpected crash](#unexpected-crash)
  - [Acknowledgements](#acknowledgements) -->


# GVP-MREP
Fast and Communication-Efficient Multi-UAV Exploration Via Voronoi Partition on Dynamic Topological Graph (IROS 2024 accepted)

# 1. Overview
**GVP-MREP** is a distributed and communication-efficient multi-UAV exploration system. For lightweight multi-UAV communication, a multi-robot dynamic topological graph (MR-DTG) is designed. Supported by MR-DTG, graph Voronoi partition (GVP) is adopted to allocate exploration tasks to each UAV. Each UAV operates GVP and optimizes trajectories to their targets distributedly.   
<p align="center">
  <img src="pics/0.gif" width = "400" height = "225"/>
  <img src="pics/1.gif" width = "400" height = "225"/>
</p>

**Video Links**: [youtube](https://www.youtube.com/watch?v=AtG9stNVjX0) or [bilibili](https://www.bilibili.com/video/BV1KC411h79h/).

**Paper**: Fast and Communication-Efficient Multi-UAV Exploration Via Voronoi Partition on Dynamic Topological Graph, Qianli Dong, Haobo Xi, Shiyong Zhang, Qingchen Bi, Tianyi Li, Ziyu Wang and Xuebo Zhang, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2024, accepted.
```
Citation not available now
```
# 2. Setup
This work is developed in Ubuntu 20.04, [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

**Prerequisites**:
```
<!-- git clone https://github.com/google/glog.git
cd glog
cmake -S . -B build -G "Unix Makefiles"
cmake --build build
cmake --build build --target test
cmake --build build --target install
# if cmake reports version error: git checkout v0.3.5 -->
cd workspace/src
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/catkin/catkin_simple.git

sudo apt install libzmqpp-dev #libgflags-dev
```

**Simulation environment**:
We adopt [RotorS](https://github.com/ethz-asl/rotors_simulator) as the simulation platform. To adapt to the scenario of multi-UAV exploration, we have made some modifications. You can get the modified version with:
```
cd workspace/src
git clone https://github.com/NKU-UAVTeam/rotors-modified.git
```

**Clone Code and Make**:
```
cd workspace/src
git clone https://github.com/NKU-MobFly-Robotics/GVP-MREP.git
cd ..
catkin_make
```
**Run Exploration**:
```
source devel/setup.bash
roslaunch murder_swarm murder_swarm_maze3.launch #large maze
#or
roslaunch murder_swarm murder_swarm_maze4.launch #small maze
```

Mamba bless your UAV!

# 3. Parameters


