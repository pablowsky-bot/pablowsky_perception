cmake_minimum_required(VERSION 3.0.2)
project(pablowsky_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/hsvBounds.cfg
)

catkin_package()
