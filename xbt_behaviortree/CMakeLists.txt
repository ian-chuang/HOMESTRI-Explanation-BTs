cmake_minimum_required(VERSION 3.0.2)
project(xbt_behaviortree)

add_compile_options(-std=c++17)

set(ROS_DEPENDENCIES
roscpp
behaviortree_cpp
behaviortree_ros
explain_bt
std_msgs
actionlib
actionlib_msgs
tf2
tf2_ros
robotiq_2f_gripper_control
moveit_msgs
geometric_shapes
controller_manager_msgs
control_msgs
xbt_planning_interface
compliant_trajectory_control
)

set(GCC_COVERAGE_LINK_FLAGS    "-lstdc++fs")

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(run_xbt src/run_xbt.cpp)
add_dependencies(run_xbt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_xbt ${catkin_LIBRARIES} ${GCC_COVERAGE_LINK_FLAGS})