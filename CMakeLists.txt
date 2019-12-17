cmake_minimum_required(VERSION 3.14)
project (Basic3D VERSION 0.0.1)

add_definitions(-DGF_PLUGIN_NAME=\"${PROJECT_NAME}\")

find_package(geoflow REQUIRED)
find_package(glm REQUIRED)

add_library(gfp_basic3d MODULE
  plugin.cpp
  nodes.cpp
)
target_link_libraries(gfp_basic3d geoflow-core glm)

set_target_properties(
  gfp_basic3d
  PROPERTIES CXX_STANDARD 17
)

install(TARGETS gfp_basic3d DESTINATION ${geoflow_PLUGIN_FOLDER})