cmake_minimum_required(VERSION 3.14)
project(articulate_model_rear_footprint_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/articulate_model_rear_footprint_publisher/articulate_model_rear_footprint_publisher.hpp
  src/articulate_model_rear_footprint_publisher.cpp
)

# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "simulation::articulate_model_rear_footprint_publisher::ArticulateModelRearFootprintPublisher"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# if(BUILD_TESTING)
# endif()

ament_auto_package(INSTALL_TO_SHARE launch)
