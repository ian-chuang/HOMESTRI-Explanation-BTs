cmake_minimum_required(VERSION 3.0.2)
project(xbt_planning_interface)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    actionlib_msgs
    message_generation
    std_msgs
    geometry_msgs
    trajectory_msgs
)

catkin_python_setup()

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SimplePlan.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  FindSurface.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    trajectory_msgs
    actionlib_msgs
)

catkin_install_python(
    PROGRAMS 
        scripts/planning_interface
        scripts/find_surface_action_server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
