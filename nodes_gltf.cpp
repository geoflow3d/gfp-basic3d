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
#include "tiny_gltf.h"

namespace geoflow::nodes::basic3d
{
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

    tinygltf::Model model;
    model.asset.generator = "Geoflow";
    model.asset.version = "2.0";
    tinygltf::Scene scene;
    tinygltf::Node node;
    tinygltf::Mesh mesh;
    tinygltf::Buffer buffer;

    // Create a simple material
    tinygltf::Material mat;
    mat.pbrMetallicRoughness.baseColorFactor = {1.0f, 0.9f, 0.9f, 1.0f};  
    mat.doubleSided = true;
    model.materials.push_back(mat);

    std::vector<arr6f> vertex_vec; // position + normal
    std::vector<unsigned> index_vec;
    std::vector<TCInfo> info_vec;
    unsigned v_offset = 0;
    unsigned i_offset = 0;
    for(unsigned i=0; i< triangle_collections_inp.size(); ++i) {
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      auto& normals = normals_inp.get<vec3f>(i);
      
      TCInfo inf;
      // compute index arrays
      {
        unsigned v_cntr = 0, i = 0 ;
        std::set<arr6f> vertex_set;
        std::map<arr6f, unsigned> vertex_map;
        for (auto &triangle : tc)
        {
          for (auto &p : triangle)
          {
            auto vertex = arr6f{p[0], p[1], p[2], normals[i][0], normals[i][1], normals[i][2]};
            auto [it, did_insert] = vertex_set.insert(vertex);
            if (did_insert)
            {
              vertex_map[vertex] = v_cntr++;
              vertex_vec.push_back(vertex);
            }
            index_vec.push_back(v_offset + vertex_map[vertex]);
            ++i;
          }
        }
        inf.vertex_offset = v_offset;
        inf.vertex_count = v_cntr;
        inf.index_offset= i_offset;
        inf.index_count = i;
        inf.index_min.push_back(v_offset);
        inf.index_max.push_back(v_offset+v_cntr-1);

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
        p = tc.box().min();
        inf.positions_min.push_back(p[0]);
        inf.positions_min.push_back(p[1]);
        inf.positions_min.push_back(p[2]);
        p = tc.box().max();
        inf.positions_max.push_back(p[0]);
        inf.positions_max.push_back(p[1]);
        inf.positions_max.push_back(p[2]);

        v_offset += v_cntr;
        i_offset += i;
      }
      info_vec.push_back(inf);
    }
    
    // TODO check if padding is needed -> no only if componentype of index and attribute has different byteSize
    auto byteOffset_attributes = index_vec.size() * sizeof(unsigned);

    for(unsigned i=0; i< triangle_collections_inp.size(); ++i) {
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
      model.bufferViews.push_back(bf_indices);

      acc_indices.bufferView = model.bufferViews.size()-1;
      acc_indices.byteOffset = 0;
      acc_indices.type = TINYGLTF_TYPE_SCALAR;
      acc_indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
      acc_indices.count = inf.index_count;
      acc_indices.minValues = {inf.index_min};
      acc_indices.maxValues = {inf.index_min};
      model.accessors.push_back(acc_indices);
      primitive.indices = model.accessors.size()-1;

      bf_attributes.buffer = 0;
      bf_attributes.byteOffset = byteOffset_attributes + inf.vertex_offset * 6 * sizeof(float);
      bf_attributes.byteLength = inf.vertex_count * 6 * sizeof(float);
      bf_attributes.byteStride = 3 * sizeof(float);
      bf_attributes.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
      model.bufferViews.push_back(bf_attributes);

      acc_positions.bufferView = model.bufferViews.size()-1;
      acc_positions.byteOffset = 0;
      acc_positions.type = TINYGLTF_TYPE_VEC3;
      acc_positions.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_positions.count = inf.vertex_count;
      acc_positions.minValues = inf.positions_min;
      acc_positions.maxValues = inf.positions_max;
      model.accessors.push_back(acc_positions);
      primitive.attributes["POSITION"] = model.accessors.size()-1;  // The index of the accessor for positions

      acc_normals.bufferView = model.bufferViews.size()-1;
      acc_normals.byteOffset = 3 * sizeof(float);
      acc_normals.type = TINYGLTF_TYPE_VEC3;
      acc_normals.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_normals.count = inf.vertex_count;
      acc_normals.minValues = inf.normals_min;
      acc_normals.maxValues = inf.normals_max;
      model.accessors.push_back(acc_normals);
      primitive.attributes["NORMAL"] = model.accessors.size()-1;  // The index of the accessor for normals

      primitive.material = 0;
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
      (unsigned char*)vertex_vec.data(), 
      vertex_vec.size()*6*sizeof(float)
    );

    model.meshes.push_back(mesh);
    node.mesh = 0;
    model.nodes.push_back(node);
    model.scenes.push_back(scene);
    model.defaultScene = 0;

    // Save it to a file
    tinygltf::TinyGLTF gltf;
    if(!gltf.WriteGltfSceneToFile(&model, manager.substitute_globals(filepath_),
                            embed_images_, // embedImages
                            embed_buffers_, // embedBuffers
                            pretty_print_, // pretty print
                            binary_) // write binary
    ) {
      throw(gfIOError("Write GLTF failed"));
    }
  }
}