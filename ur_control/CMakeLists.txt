cmake_minimum_required(VERSION 3.1.3)
project(ur_control)

find_package(catkin REQUIRED
COMPONENTS
  geometry_msgs
  sensor_msgs
  ur_msgs
  shape_msgs
  std_msgs
  moveit_msgs
  moveit_commander
)

catkin_python_setup()

catkin_package()


