cmake_minimum_required(VERSION 3.22)

project(whycode)

#####################
## Global settings ##
#####################

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -ggdb -g)
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(whycode_interfaces REQUIRED)

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

include_directories(include)

add_executable(whycode_node
    src/whycode_ros/whycode_ros_node.cpp
    src/whycode_ros/CWhycodeROSNode.cpp
    src/whycode/CWhycode.cpp
    src/whycode/CCircleDetect.cpp
    src/whycode/CRawImage.cpp
    src/whycode/CTransformation.cpp
    src/whycode/CNecklace.cpp
    )

ament_target_dependencies(whycode_node rclcpp cv_bridge std_msgs sensor_msgs geometry_msgs image_transport whycode_interfaces OpenCV)

#############
## Install ##
#############

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS whycode_node 
  DESTINATION lib/${PROJECT_NAME}
)

###########
## Setup ##
###########

ament_package()
