cmake_minimum_required(VERSION 2.8.3)
project(prox_ray_gazebo_classic)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
# Create the libProximityRayPlugin.so library.
set(proximity_ray_plugin_name ProximityRayPlugin)
add_library(${proximity_ray_plugin_name} SHARED src/ProximityRayPlugin.cc)
target_link_libraries(${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)

