cmake_minimum_required(VERSION 2.8.3)
project(paper_glider)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        pcl_ros
        pcl_conversions
        )

catkin_package(
    CATKIN_DEPENDS moveit_core moveit_visual_tools moveit_ros_planning_interface
    )


## Build
# include_directories()


## Install
install(DIRECTORY launch meshes scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
