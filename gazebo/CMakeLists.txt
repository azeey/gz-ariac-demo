cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(prox_ray_gazebo)

find_package(ignition-gazebo6 REQUIRED)

# Create the libProximityRayPlugin.so library.
set(proximity_ray_plugin_name ProximityRayPlugin)
add_library(${proximity_ray_plugin_name} SHARED src/ProximityRayPlugin.cc)
target_link_libraries(${proximity_ray_plugin_name}
  PRIVATE ignition-gazebo6::core
)

