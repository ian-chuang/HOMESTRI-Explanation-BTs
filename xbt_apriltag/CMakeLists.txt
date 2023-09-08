cmake_minimum_required(VERSION 3.0.2)
project(xbt_apriltag)

find_package(catkin REQUIRED COMPONENTS 
    rospy
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
)

catkin_install_python(
    PROGRAMS 
        scripts/transform_publisher
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
