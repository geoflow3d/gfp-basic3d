#include "nodes.hpp"

using namespace geoflow::nodes::basic3d;

void register_nodes(geoflow::NodeRegister& node_register) {
  node_register.register_node<OBJWriterNode>("OBJWriter");
  node_register.register_node<VecOBJWriterNode>("OBJVecWriter");
}