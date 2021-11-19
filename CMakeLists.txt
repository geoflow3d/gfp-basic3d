cmake_minimum_required(VERSION 3.14)
project (CoreIO VERSION 0.0.1)

find_package(geoflow REQUIRED)

include_directories(${geoflow_INCLUDE_DIRS})

set(GF_PLUGIN_NAME ${PROJECT_NAME})
set(GF_PLUGIN_TARGET_NAME "gfp_core_io")
set(GF_PLUGIN_REGISTER ${PROJECT_SOURCE_DIR}/register.hpp)
geoflow_create_plugin(
  nodes_obj.cpp
  nodes_ply.cpp
  nodes_cityjson.cpp
)

target_link_libraries(gfp_core_io PRIVATE geoflow-core)
