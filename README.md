# paper_glider :small_airplane:
Sample package for using a 6-axis robotic arm to launch a paper glider. Intended to support the development of robotic applications at the Artifically Intelligent Manufacturing Systems (AIMS) Lab at The Ohio State University.

_Note: This repository was designed for ROS Melodic. It has not been tested on other distributions._
_Specifically designed for the Motoman MH5 robot as supported by the ROS-Industrial program._

## Installation
The following section explains how to setup the package to work.

### Prerequisites
  - **ROS Melodic:** For obtaining and configuring ROS follow the installation instructions for [full-desktop Melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu).
  - **Catkin workspace:** Create a clean [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/index.html) workspace.
  - **MoveIt 1:** For installation instructions see [MoveIt's webpage](https://moveit.ros.org/install/).

### Required Repositories
  Clone the below repositories into your catkin-tools workspace:
  - [paper_glider](https://github.com/osu-aims/paper_glider)
  - [ros-industrial/Motoman](https://github.com/ros-industrial/motoman)


### Dependencies
To automatically install any missing dependencies of your ROS installation, run the following terminal commands:

```
#---------- install third party dependencies -----------
sudo apt-get update
# Move to the root of the workspace
cd [PATH/TO/YOUR/WORKSPACE]
# Install all dependencies of packages in the workspace
rosdep install --from-paths src --ignore-src -r -y
# Build your workspace
catkin build
source devel/setup.bash
```
Once the workspace build process is completed you are ready to start playing...cheers!

#### Install missing dependencies
If the build fails, it occurs usually to missing package dependencies or missing third party (non-ros) packages. When this occurs the build log in the terminal indicates the name of the package dependency that it is missing, then try:

```
sudo apt-get update ros-kinetic-[package-name]
# separate the package name words with a '-'
```
If a package is not found it is probably a third-party dependency, google the name of the package and search for installation instructions:

## Usage

```
roslaunch motoman_mh5_moveit_config demo.roslaunch
rosrun paper_glider paper_glider.py
```
