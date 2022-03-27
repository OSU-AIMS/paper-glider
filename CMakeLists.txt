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
        roslib
        descartes_core
        descartes_moveit
        descartes_trajectory
        descartes_planner
        descartes_utilities
        trajectory_msgs
        tf
        tf_conversions
        eigen_conversions
        )

catkin_package(
    CATKIN_DEPENDS moveit_core moveit_visual_tools moveit_ros_planning_interface
    )



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_demo_descartes demo_descartes/descartes_example.cpp)

target_link_libraries(${PROJECT_NAME}_demo_descartes
    ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_demo_descartes
    PROPERTIES OUTPUT_NAME demo_descartes
    PREFIX "")



#############
## Install ##
#############

install(DIRECTORY launch meshes scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS ${PROJECT_NAME}_demo_descartes
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
