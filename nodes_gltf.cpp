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

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"

namespace geoflow::nodes::basic3d
{
  void create_materials(tinygltf::Model& model) {
     //building red
    tinygltf::Material building;
    building.pbrMetallicRoughness.baseColorFactor = { 1.000, 0.000, 0.000, 1.0 };
    model.materials.push_back(building);
    // terrain brown
    tinygltf::Material terrain;
    terrain.pbrMetallicRoughness.baseColorFactor = { 0.588, 0.403, 0.211, 1.0 };
    model.materials.push_back(terrain);
     // transport grey
    tinygltf::Material  transport;
     transport.pbrMetallicRoughness.baseColorFactor = { 0.631, 0.607, 0.592, 1.0 };
     model.materials.push_back(transport);
    // waterbody blue
    tinygltf::Material waterbody;
    waterbody.pbrMetallicRoughness.baseColorFactor = { 0.070, 0.949, 0.972, 1.0 };
    model.materials.push_back(waterbody);
    // vegetation green
    tinygltf::Material vegetation;
    vegetation.pbrMetallicRoughness.baseColorFactor = { 0.000, 1.000, 0.000, 1.0 };
    model.materials.push_back(vegetation);
     // landuse yellow
    tinygltf::Material  landuse;
     landuse.pbrMetallicRoughness.baseColorFactor = { 0.909, 0.945, 0.196, 1.0 };
     model.materials.push_back(landuse);
     // CityFurniture orange
    tinygltf::Material  CityFurniture;
     CityFurniture.pbrMetallicRoughness.baseColorFactor = { 0.894, 0.494, 0.145, 1.0 };
     model.materials.push_back(CityFurniture);
    // bridge purple
    tinygltf::Material bridge;
    bridge.pbrMetallicRoughness.baseColorFactor = { 0.466, 0.094, 0.905, 1.0 };
    model.materials.push_back(bridge);
    // tunnel black
    tinygltf::Material tunnel;
    tunnel.pbrMetallicRoughness.baseColorFactor = { 0.011, 0.011, 0.007, 1.0 };
    model.materials.push_back(tunnel);
    // GenericCityObject pink
    tinygltf::Material GenericCityObject;
    GenericCityObject.pbrMetallicRoughness.baseColorFactor = { 0.909, 0.188, 0.827, 1.0 };
    model.materials.push_back(GenericCityObject);
  };

  unsigned get_material_id(const std::string type) {
    std::cout << "assigning material for " << type << std::endl;
    if (type == "Building" or type == "BuildingPart" or type == "BuildingInstallation")
      return 0;
    else if (type == "TINRelief")
      return 1;
    else if (type == "Road" or type == "Railway" or type == "TransportSquare")
      return 2;
    else if (type == "WaterBody")
      return 3;
    else if (type == "PlantCover" or type == "SolitaryVegetationObject")
      return 4;
    else if (type == "LandUse")
      return 5;
    else if (type == "CityFurniture")
      return 6;
    else if (type == "Bridge" or type == "BridgePart" or type == "BridgeInstallation" or type == "BridgeConstructionElement")
      return 7;
    else if (type == "Tunnel" or type == "TunnelPart" or type == "TunnelInstallation")
      return 8;
    else if (type == "GenericCityObject")
      return 9;
    return 0;
  }

  void GLTFWriterNode::process() {
    typedef std::array<float,6> arr6f;
    struct TCInfo {
      unsigned vertex_offset;
      unsigned vertex_count;
      unsigned index_offset;
      unsigned index_count;
      std::vector<double> index_min;
      std::vector<double> index_max;
      std::vector<double> normals_min;
      std::vector<double> positions_min;
      std::vector<double> normals_max;
      std::vector<double> positions_max;
    };

    auto& triangle_collections_inp = vector_input("triangles");
    auto& normals_inp = vector_input("normals");
    auto& featuretype_inp = vector_input("feature_type");

    tinygltf::Model model;
    model.asset.generator = "Geoflow";
    model.asset.version = "2.0";
    tinygltf::Scene scene;
    tinygltf::Node node;
    tinygltf::Mesh mesh;
    tinygltf::Buffer buffer;

    // Create a simple material
    // tinygltf::Material mat;
    // mat.pbrMetallicRoughness.baseColorFactor = {1.0f, 0.9f, 0.9f, 1.0f};  
    // mat.doubleSided = true;
    // model.materials.push_back(mat);
    create_materials(model);

    std::vector<arr6f> vertex_vec; // position + normal
    std::vector<unsigned> index_vec;
    std::vector<TCInfo> info_vec;
    unsigned v_offset = 0;
    unsigned i_offset = 0;
    manager.set_rev_crs_transform(manager.substitute_globals(CRS_).c_str());
    for(unsigned i=0; i< triangle_collections_inp.size(); ++i) {
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0)
        continue;
      auto& normals = normals_inp.get<vec3f>(i);
      
      
      TCInfo inf;
      Box positions_box;
      // compute arrays
      {
        // count the nr of vertices and indices stored for this tc
        unsigned v_cntr = 0, i = 0 ;
        std::set<arr6f> vertex_set;
        std::map<arr6f, unsigned> vertex_map;
        for (auto &triangle : tc)
        {
          for (auto &p : triangle)
          {
            const auto p_ = manager.coord_transform_rev(p);
            // NB: narrowing double to float here, not ideal
            auto vertex = arr6f{
              float(p_[0]), 
              float(p_[1]), 
              float(p_[2]), 
              normals[i][0], normals[i][1], normals[i][2]
            };
            auto [it, did_insert] = vertex_set.insert(vertex);
            if (did_insert)
            {
              vertex_map[vertex] = v_cntr++;
              vertex_vec.push_back(vertex);
              positions_box.add(p_);
            }
            index_vec.push_back(vertex_map[vertex]);
            ++i;
          }
        }
        inf.vertex_offset = v_offset;
        inf.vertex_count = v_cntr;
        inf.index_offset= i_offset;
        inf.index_count = i;
        inf.index_min.push_back(0);
        inf.index_max.push_back(v_cntr-1);

        Box normals_box;
        normals_box.add(normals);
        auto p = normals_box.min();
        inf.normals_min.push_back(p[0]);
        inf.normals_min.push_back(p[1]);
        inf.normals_min.push_back(p[2]);
        p = normals_box.max();
        inf.normals_max.push_back(p[0]);
        inf.normals_max.push_back(p[1]);
        inf.normals_max.push_back(p[2]);
        p = positions_box.min();
        inf.positions_min.push_back(p[0]);
        inf.positions_min.push_back(p[1]);
        inf.positions_min.push_back(p[2]);
        p = positions_box.max();
        inf.positions_max.push_back(p[0]);
        inf.positions_max.push_back(p[1]);
        inf.positions_max.push_back(p[2]);

        v_offset += v_cntr;
        i_offset += i;
      }
      info_vec.push_back(inf);
    }
    manager.clear_rev_crs_transform();
    
    // TODO check if padding is needed -> no only if componentype of index and attribute has different byteSize
    auto byteOffset_attributes = index_vec.size() * sizeof(unsigned);

    for(unsigned i=0; i< info_vec.size(); ++i) {
      auto& ftype = featuretype_inp.get<std::string>(i);
      auto& inf = info_vec[i];
      tinygltf::BufferView bf_attributes;
      tinygltf::BufferView bf_indices;
      tinygltf::Accessor acc_positions;
      tinygltf::Accessor acc_normals;
      tinygltf::Accessor acc_indices;
      tinygltf::Primitive primitive;

      bf_indices.buffer = 0;
      bf_indices.byteOffset = inf.index_offset * sizeof(unsigned);
      bf_indices.byteLength = inf.index_count * sizeof(unsigned);
      bf_indices.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
      auto id_bf_indices = model.bufferViews.size();
      model.bufferViews.push_back(bf_indices);

      acc_indices.bufferView = id_bf_indices;
      acc_indices.byteOffset = 0;
      acc_indices.type = TINYGLTF_TYPE_SCALAR;
      acc_indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
      acc_indices.count = inf.index_count;
      acc_indices.minValues = {inf.index_min};
      acc_indices.maxValues = {inf.index_max};
      model.accessors.push_back(acc_indices);
      primitive.indices = model.accessors.size()-1;

      bf_attributes.buffer = 0;
      bf_attributes.byteOffset = byteOffset_attributes + inf.vertex_offset * 6 * sizeof(float);
      bf_attributes.byteLength = inf.vertex_count * 6 * sizeof(float);
      bf_attributes.byteStride = 6 * sizeof(float);
      bf_attributes.target = TINYGLTF_TARGET_ARRAY_BUFFER;
      auto id_bf_attributes = model.bufferViews.size();
      model.bufferViews.push_back(bf_attributes);

      acc_positions.bufferView = id_bf_attributes;
      acc_positions.byteOffset = 0;
      acc_positions.type = TINYGLTF_TYPE_VEC3;
      acc_positions.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_positions.count = inf.vertex_count;
      acc_positions.minValues = inf.positions_min;
      acc_positions.maxValues = inf.positions_max;
      model.accessors.push_back(acc_positions);
      primitive.attributes["POSITION"] = model.accessors.size()-1;  // The index of the accessor for positions

      acc_normals.bufferView = id_bf_attributes;
      acc_normals.byteOffset = 3 * sizeof(float);
      acc_normals.type = TINYGLTF_TYPE_VEC3;
      acc_normals.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_normals.count = inf.vertex_count;
      acc_normals.minValues = inf.normals_min;
      acc_normals.maxValues = inf.normals_max;
      model.accessors.push_back(acc_normals);
      primitive.attributes["NORMAL"] = model.accessors.size()-1;  // The index of the accessor for normals

      primitive.material = get_material_id(ftype);
      primitive.mode = TINYGLTF_MODE_TRIANGLES;
      mesh.primitives.push_back(primitive);
    }

    buffer.data.resize(index_vec.size()*sizeof(unsigned) + vertex_vec.size()*6*sizeof(float));
    memcpy(
      buffer.data.data(), 
      (unsigned char*)index_vec.data(), 
      index_vec.size()*sizeof(unsigned)
    );
    memcpy(
      buffer.data.data() + index_vec.size()*sizeof(unsigned), 
      (unsigned char*)vertex_vec[0].data(), 
      vertex_vec.size()*6*sizeof(float)
    );

    // std::cout << std::endl;
    // for(unsigned i=0; i<index_vec.size(); ++i) {
    //   std::cout << i << " : "<< index_vec[i] << std::endl;
    // }
    // for(unsigned i=0; i<vertex_vec.size(); ++i) {
    //   std::cout << i << " : "<< vertex_vec[i][1] << ", " << vertex_vec[i][1] << ", " << vertex_vec[i][2] << " | ";
    //   std::cout << vertex_vec[i][3] << ", " << vertex_vec[i][4] << ", " << vertex_vec[i][5] << std::endl;
    // }
    // std::cout << "buffer size=" << buffer.data.size() << std::endl;
    // for (size_t i=0; i<index_vec.size()*sizeof(unsigned); i+=4) {
    //   std::cout << i/4 << " : \n";
    //   // if (i<36*4) {
    //     unsigned u;
    //     memcpy(&u, &buffer.data[i], sizeof(u));
    //     std::cout << "u = " << u << std::endl;
    //   // } else {
    //     // for (size_t j=i; j<i+12; j+=4){
    //       // float f;
    //       // memcpy(&f, &buffer.data[i], sizeof(f));
    //       // std::cout << "f = " << f << std::endl;
    //     // }
    //     // std::cout << std::endl;
    //   // }
    // }

    model.meshes.push_back(mesh);
    node.mesh = 0;
    // Apply z-up to y-up transformation since our data is z-up, but gltf requires y-up (per 3D tiles specs recommendation)
    // see https://github.com/CesiumGS/3d-tiles/tree/main/specification#y-up-to-z-up
    node.matrix = {1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1};
    model.nodes.push_back(node);
    scene.nodes.push_back(0);
    model.scenes.push_back(scene);
    model.defaultScene = 0;
    model.buffers.push_back(buffer);

    // Save it to a file
    fs::path fname = fs::path(manager.substitute_globals(filepath_));
    fs::create_directories(fname.parent_path());
    tinygltf::TinyGLTF gltf;
    if(!gltf.WriteGltfSceneToFile(&model, fname,
                            embed_images_, // embedImages
                            embed_buffers_, // embedBuffers
                            pretty_print_, // pretty print
                            binary_) // write binary
    ) {
      throw(gfIOError("Write GLTF failed"));
    }
  }
}