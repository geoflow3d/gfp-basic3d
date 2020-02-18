cmake_minimum_required(VERSION 3.14)
project (Basic3D VERSION 0.0.1)

add_definitions(-DGF_PLUGIN_NAME=\"${PROJECT_NAME}\")

find_package(geoflow REQUIRED)
find_package(glm REQUIRED)
add_subdirectory(thirdparty/poly2tri)

set(GF_PLUGIN_NAME ${PROJECT_NAME})
set(GF_PLUGIN_TARGET_NAME "gfp_basic3d")
set(GF_PLUGIN_REGISTER ${PROJECT_SOURCE_DIR}/register.hpp)
geoflow_create_plugin(
  nodes.cpp
)

target_link_libraries(gfp_basic3d geoflow-core glm poly2tri)
