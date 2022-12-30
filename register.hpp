// This file is part of gfp-basic3d
// Copyright (C) 2018-2022 Ravi Peters

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "nodes.hpp"

using namespace geoflow::nodes::basic3d;

void register_nodes(geoflow::NodeRegister& node_register) {
  node_register.register_node<OBJWriterNode>("OBJWriter");
  node_register.register_node<PLYWriterNode>("PLYWriter");
  node_register.register_node<VecOBJWriterNode>("OBJVecWriter");
  // node_register.register_node<CityJSONReaderNode>("CityJSONReader");
  node_register.register_node<CityJSONWriterNode>("CityJSONWriter");
  node_register.register_node<CityJSONFeatureWriterNode>("CityJSONFeatureWriter");
  node_register.register_node<CityJSONFeatureMetadataWriterNode>("CityJSONFeatureMetadataWriter");
  node_register.register_node<JSONReaderNode>("JSONReader");
  node_register.register_node<CityJSONLinesWriterNode>("CityJSONLinesWriter");
  // node_register.register_node<Mesh2CityGMLWriterNode>("Mesh2CityGMLWriter");
  node_register.register_node<CityJSONL2MeshNode>("CityJSONL2Mesh");
}