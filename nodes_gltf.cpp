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
#include <limits>

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"
#include <nlohmann/json.hpp>

namespace geoflow::nodes::basic3d
{
  typedef std::array<float,6> arr6f;
  typedef std::array<float,7> arr7f;
  struct IntermediateDataHelper {
    // struct FeatureAttributeData {
    //   std::vector<unsigned> indices;
    //   std::vector<arr3f> positions;
    //   std::vector<arr3f> normals;
    //   std::vector<float> feature_ids;
    // };
    std::unordered_map<std::string, arr7f> data;
    // std::unordered_map<std::string, FeatureAttributeData> data;
    std::unordered_map<std::string, unsigned> ftype_counts;

    arr3f center_point;
    NodeManager& manager;
    bool relative_to_center;

    float feature_id_cnt = 0.0;
    size_t total_count;

    IntermediateDataHelper(
      const arr3f& center_point,
      NodeManager& manager,
      bool& relative_to_center
    ) : center_point(center_point), manager(manager), relative_to_center(relative_to_center)
    {
    }

    void add_feature(
      const std::string& feature_type,
      const TriangleCollection& tc, 
      const vec3f& normals
    ) {
      
      // count the nr of vertices and indices stored for this tc
      std::set<arr6f> vertex_set;
      std::map<arr6f, unsigned> vertex_map;
      size_t index_offset = 0, i = 0, v_cntr = 0;
      if(data.count(feature_type)) index_offset = data[feature_type].indices.size();
      for (auto &triangle : tc)
      {
        for (auto &p : triangle)
        {
          auto p_ = manager.coord_transform_rev(p);
          // NB: narrowing double to float here, not ideal
          arr6f vertex;
          if(relative_to_center) {
            p_[0] -= float(center_point[0]); 
            p_[1] -= float(center_point[1]);
            p_[2] -= float(center_point[2]);
          }
          vertex = arr6f{
            float(p_[0]), 
            float(p_[1]), 
            float(p_[2]), 
            normals[i][0], normals[i][1], normals[i++][2]
          };
          auto [it, did_insert] = vertex_set.insert(vertex);
          if (did_insert)
          {
            vertex_map[vertex] = v_cntr++;
            
            // reproject normal
            arr3f pn_{p[0]+vertex[3], p[1]+vertex[4], p[2]+vertex[5]};
            auto pn = manager.coord_transform_rev(pn_);
            arr3f n{float(pn[0]-p_[0]), float(pn[1]-p_[1]), float(pn[2]-p_[2])};
            auto l = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
            n[0] = n[0]/l;
            n[1] = n[1]/l;
            n[2] = n[2]/l;

            // write to vectors
            // data[feature_type].positions.push_back(arr3f{p[0], p[1], p[2]});
            // data[feature_type].normals.push_back(n);
            // data[feature_type].feature_ids.push_back(feature_id_cnt);

            data[feature_type].push_back(p[0]);
            data[feature_type].push_back(p[1]);
            data[feature_type].push_back(p[2]);
            data[feature_type].push_back(n[0]);
            data[feature_type].push_back(n[1]);
            data[feature_type].push_back(n[2]);
            data[feature_type].push_back(feature_id_cnt);
            ++total_count;
            // positions_box.add(arr3f{vertex[0], vertex[1], vertex[2]});
          }
          data[feature_type].indices.push_back(index_offset + vertex_map[vertex]);
        }
      }
      feature_id_cnt += 1.0;
      if (ftype_counts.count(feature_type)) {
        ftype_counts[feature_type] += 1;
      } else {
        ftype_counts[feature_type] = 0;
      }
    }
  };

  void set_min_max(tinygltf::Accessor& accessor, std::vector<arr3f>& vec) {
    Box box;
    box.add(vec);
    auto p = box.min();
    accessor.minValues.push_back(p[0]);
    accessor.minValues.push_back(p[1]);
    accessor.minValues.push_back(p[2]);
    p = box.max();
    accessor.maxValues.push_back(p[0]);
    accessor.maxValues.push_back(p[1]);
    accessor.maxValues.push_back(p[2]);
  }

  struct GLTFBuilder {

    tinygltf::Model model;
    tinygltf::Mesh mesh;
    tinygltf::Buffer buffer;
    
    GLTFBuilder(){
      model.asset.generator = "Geoflow";
      model.asset.version = "2.0";
    }

    tinygltf::Value create_ext_mesh_features(int                featureCount,
                                             int                attribute,
                                             int                propertyTable,
                                             const std::string& label)
    { // EXT_mesh_features
      // We only have one feature ID set. This feature ID set releates the
      // feature ID vertex attribute to the feature attribute, thus each for a
      // given feature (eg CityObject), we have one feature ID and this added to
      // each vertex (this is the vertex attribute).
      tinygltf::Value::Object featureId_set;
      // The number of unique features that are identified WITHIN ONE array of
      // feature ID-s for ONE mesh primitive. Thus, if we are assigning attributes
      // to a whole CityObject for instance, which is represented by one mesh
      // primitive, then this value will be 1.
      // If we need to distinguish between semantic surfaces, then this value is
      // the number of distinct surfaces within the mesh primitive.
      featureId_set["featureCount"] = tinygltf::Value(featureCount);
      // The value of "attribute" value is the index of the standard gltf
      // accessor that refers to the actual feature ID-s in the buffer, for the
      // current feature. The _FEATURE_ID_n semantic has the same value for 'n'
      // as the "attribute", thus the index of the accessor.
      featureId_set["attribute"] = tinygltf::Value(attribute);
      // This feature ID set is associated with the propertyTable at the index.
      featureId_set["propertyTable"] = tinygltf::Value(propertyTable);
      if (!label.empty()) {
        featureId_set["label"] = tinygltf::Value(label);
      }
      tinygltf::Value::Array featureId_sets;
      featureId_sets.emplace_back(featureId_set);
      tinygltf::Value::Object ext_mesh_features_object;
      ext_mesh_features_object["featureIds"] = tinygltf::Value(featureId_sets);
      return tinygltf::Value(ext_mesh_features_object);
    }

    size_t create_material(std::string type) {
      //building red
      if (type == "Building" or type == "BuildingPart" or type == "BuildingInstallation") {
        tinygltf::Material building;
        building.pbrMetallicRoughness.baseColorFactor = { 1.000, 0.000, 0.000, 1.0 };
        model.materials.push_back(building);
      }
      // terrain brown
      else if (type == "TINRelief") {
        tinygltf::Material terrain;
        terrain.pbrMetallicRoughness.baseColorFactor = { 0.588, 0.403, 0.211, 1.0 };
        model.materials.push_back(terrain);
      }
      // transport grey
      else if (type == "Road" or type == "Railway" or type == "TransportSquare") {
        tinygltf::Material  transport;
        transport.pbrMetallicRoughness.baseColorFactor = { 0.631, 0.607, 0.592, 1.0 };
        model.materials.push_back(transport);
      }
      // waterbody blue
      else if (type == "WaterBody") {
        tinygltf::Material waterbody;
        waterbody.pbrMetallicRoughness.baseColorFactor = { 0.070, 0.949, 0.972, 1.0 };
        model.materials.push_back(waterbody);
      }
      // vegetation green
      else if (type == "PlantCover" or type == "SolitaryVegetationObject") {
        tinygltf::Material vegetation;
        vegetation.pbrMetallicRoughness.baseColorFactor = { 0.000, 1.000, 0.000, 1.0 };
        model.materials.push_back(vegetation);
      }
      // landuse yellow
      else if (type == "LandUse") {
        tinygltf::Material  landuse;
        landuse.pbrMetallicRoughness.baseColorFactor = { 0.909, 0.945, 0.196, 1.0 };
        model.materials.push_back(landuse);
      }
      // CityFurniture orange
      else if (type == "CityFurniture") {
        tinygltf::Material  CityFurniture;
        CityFurniture.pbrMetallicRoughness.baseColorFactor = { 0.894, 0.494, 0.145, 1.0 };
        model.materials.push_back(CityFurniture);
      }
      // bridge purple
      else if (type == "Bridge" or type == "BridgePart" or type == "BridgeInstallation" or type == "BridgeConstructionElement") {
        tinygltf::Material bridge;
        bridge.pbrMetallicRoughness.baseColorFactor = { 0.466, 0.094, 0.905, 1.0 };
        model.materials.push_back(bridge);
      }
      // tunnel black
      else if (type == "Tunnel" or type == "TunnelPart" or type == "TunnelInstallation") {
        tinygltf::Material tunnel;
        tunnel.pbrMetallicRoughness.baseColorFactor = { 0.011, 0.011, 0.007, 1.0 };
        model.materials.push_back(tunnel);
      }
      // GenericCityObject pink
      else if (type == "GenericCityObject") {
        tinygltf::Material GenericCityObject;
        GenericCityObject.pbrMetallicRoughness.baseColorFactor = { 0.909, 0.188, 0.827, 1.0 };
        model.materials.push_back(GenericCityObject);
      }
      return model.materials.size()-1;
    }

    size_t copy_to_buffer(unsigned char* src, const size_t& element_byteSize, const size_t& count, const size_t& byteOffset) {
      size_t byteLength = count * element_byteSize;
      buffer.data.resize( buffer.data.size() + byteLength );
      // add padding if needed
      // buffer.data.push_back('\0')
      memcpy(
        buffer.data.data() + byteOffset,
        src,
        byteLength
      );
      return byteLength;
    }

    void add_geometry(IntermediateDataHelper& IDH) {
      size_t buffer_idx = 0;
      size_t bufferLength = 0;
      size_t byteOffset = 0;
      size_t feature_id_set_idx = 0;
      for (auto& [ftype, data] : IDH.data) {
        tinygltf::Primitive  primitive;
        tinygltf::BufferView bf_positions;
        tinygltf::BufferView bf_normals;
        tinygltf::BufferView bf_indices;
        tinygltf::BufferView bf_feature_ids;
        tinygltf::Accessor   acc_positions;
        tinygltf::Accessor   acc_normals;
        tinygltf::Accessor   acc_indices;
        tinygltf::Accessor   acc_feature_ids;
        
        // indices
        auto [min, max] = std::minmax_element(begin(data.indices), end(data.indices));
        size_t element_byteSize = sizeof(unsigned);
        size_t element_count = data.indices.size();
        size_t byteLength = 0;
        // if (! (*max <= std::numeric_limits<unsigned short>::max())) {
        //   element_byteSize = sizeof(unsigned short);
        // }
        if (element_byteSize == sizeof(unsigned short)) {
          std::vector<unsigned short> part( data.indices.begin(), data.indices.end() );
          byteLength = copy_to_buffer((unsigned char*)part.data(), element_byteSize, element_count, byteOffset);
        } else { //if (max <= numeric_limits<unsigned int>::max()) {
          byteLength = copy_to_buffer((unsigned char*)data.indices.data(), element_byteSize, element_count, byteOffset);
        }

        bf_indices.buffer     = buffer_idx;
        bf_indices.byteOffset = byteOffset;
        bf_indices.byteLength = byteLength;
        bf_indices.target     = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
        auto id_bf_indices    = model.bufferViews.size();
        model.bufferViews.push_back(bf_indices);
        byteOffset += byteLength;
        
        acc_indices.bufferView    = id_bf_indices;
        // acc_indices.byteOffset    = 0;
        acc_indices.type          = TINYGLTF_TYPE_SCALAR;
        acc_indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
        acc_indices.count         = element_count;
        acc_indices.minValues     = { double(*min) };
        acc_indices.maxValues     = { double(*max) };
        primitive.indices = model.accessors.size();
        model.accessors.push_back(acc_indices);
        
        // positions
        element_byteSize = 3 * sizeof(float);
        element_count = data.positions.size();
        byteLength = copy_to_buffer((unsigned char*)data.positions.data(), element_byteSize, element_count, byteOffset);

        bf_positions.buffer     = buffer_idx;
        bf_positions.byteOffset = byteOffset;
        bf_positions.byteLength = byteLength;
        bf_positions.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
        auto id_bf_positions    = model.bufferViews.size();
        model.bufferViews.push_back(bf_positions);
        byteOffset += byteLength;

        acc_positions.bufferView    = id_bf_positions;
        // acc_positions.byteOffset    = 0;
        acc_positions.type          = TINYGLTF_TYPE_VEC3;
        acc_positions.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        acc_positions.count         = element_count;
        set_min_max(acc_positions, data.positions);
        primitive.attributes["POSITION"] = model.accessors.size(); // The index of the accessor for positions
        model.accessors.push_back(acc_positions);
        
        // normals
        element_byteSize = 3 * sizeof(float);
        element_count = data.normals.size();
        byteLength = copy_to_buffer((unsigned char*)data.normals.data(), element_byteSize, element_count, byteOffset);

        bf_normals.buffer     = buffer_idx;
        bf_normals.byteOffset = byteOffset;
        bf_normals.byteLength = byteLength;
        bf_normals.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
        auto id_bf_normals    = model.bufferViews.size();
        model.bufferViews.push_back(bf_normals);
        byteOffset += byteLength;

        acc_normals.bufferView    = id_bf_normals;
        // acc_normals.byteOffset    = 0;
        acc_normals.type          = TINYGLTF_TYPE_VEC3;
        acc_normals.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        acc_normals.count         = element_count;
        set_min_max(acc_normals, data.normals);
        primitive.attributes["NORMAL"] = model.accessors.size(); // The index of the accessor for normals
        model.accessors.push_back(acc_normals);
        
        // feature_ids
        element_byteSize = sizeof(float);
        element_count = data.feature_ids.size();
        byteLength = copy_to_buffer((unsigned char*)data.feature_ids.data(), element_byteSize, element_count, byteOffset);

        bf_feature_ids.buffer     = buffer_idx;
        bf_feature_ids.byteOffset = byteOffset;
        bf_feature_ids.byteLength = byteLength;
        bf_feature_ids.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
        auto id_bf_feature_ids    = model.bufferViews.size();
        model.bufferViews.push_back(bf_feature_ids);
        byteOffset += byteLength;

        acc_feature_ids.bufferView    = id_bf_feature_ids;
        // acc_feature_ids.byteOffset    = 0;
        acc_feature_ids.type          = TINYGLTF_TYPE_SCALAR;
        acc_feature_ids.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        acc_feature_ids.count         = element_count;
        {
          auto [min, max] = std::minmax_element(begin(data.feature_ids), end(data.feature_ids));
          acc_feature_ids.minValues     = { *min };
          acc_feature_ids.maxValues     = { *max };
        }
        std::string fid = "_FEATURE_ID_" + std::to_string(feature_id_set_idx );
        primitive.attributes[fid] = model.accessors.size(); // The index of the accessor for feature ids
        model.accessors.push_back(acc_feature_ids);

        tinygltf::ExtensionMap primitive_extensions;
        primitive_extensions["EXT_mesh_features"] = create_ext_mesh_features(
          IDH.ftype_counts[ftype], feature_id_set_idx, 0, ""); // set to "" the feature_id_attribute_

        primitive.material   = create_material(ftype);
        primitive.mode       = TINYGLTF_MODE_TRIANGLES;
        primitive.extensions = primitive_extensions;
        mesh.primitives.push_back(primitive);
      }
    }

    void finalise(bool relative_to_center, const arr3f centerpoint) {

      // add buffer and mesh objects to model
      model.buffers.push_back(buffer);
      model.meshes.push_back(mesh);

      // add a scene and a node
      tinygltf::Scene scene;
      tinygltf::Node node;
      node.mesh = 0;
      // Apply z-up to y-up transformation since our data is z-up, but gltf requires y-up (per 3D tiles specs recommendation)
      // see https://github.com/CesiumGS/3d-tiles/tree/main/specification#y-up-to-z-up
      // matrices are in column major order
      if(relative_to_center) {
        // also include a translation by multiplication with translation matrix. ie
        //  | 1  0  0  0 |   | 1 0 0 c0 |
        //  | 0  0  1  0 |   | 0 1 0 c1 |
        //  | 0 -1  0  0 | . | 0 0 1 c2 |
        //  | 0  0  0  1 |   | 0 0 0 1  |
        node.matrix = {
          1,  0,  0,  0,
          0,  0, -1,  0,
          0,  1,  0,  0,
          centerpoint[0],  centerpoint[2], -centerpoint[1],  1
        };
      } else {
        node.matrix = {
          1,  0,  0,  0,
          0,  0, -1,  0,
          0,  1,  0,  0,
          0,  0,  0,  1
        };
      }
      model.nodes.push_back(node);
      scene.nodes.push_back(0);
      model.scenes.push_back(scene);
      model.defaultScene = 0;

      // extensions
      model.extensionsUsed.emplace_back("EXT_mesh_features");
    }
  };

  void GLTFWriterNode::process() {

    // inputs
    auto& triangle_collections_inp = vector_input("triangles");
    auto& normals_inp = vector_input("normals");
    auto& featuretype_inp = vector_input("feature_type");
    auto& attributes_inp = poly_input("attributes");

    // set CRS
    manager.set_rev_crs_transform(manager.substitute_globals(CRS_).c_str());

    // determine approximate centerpoint
    Box global_bbox;
    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0)
        continue;
      global_bbox.add(manager.coord_transform_rev(tc[0][1]));
    }
    arr3f centerpoint = global_bbox.center();

    // create intermediate vectors
    IntermediateDataHelper iData(centerpoint, manager, relative_to_center);

    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      const auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0)
        continue;

      const auto& normals = normals_inp.get<vec3f>(i);
      const std::string& ftype = featuretype_inp.get<std::string>(i);
      iData.add_feature(
        ftype,
        tc,
        normals
      );
    }

    // clear CRS; we are done reading coordinates
    manager.clear_rev_crs_transform();

    if (iData.total_count == 0) {
      std::cout<<"no vertices to write, aborting...\n";
      return;
    }

    // build the gltf
    GLTFBuilder gltf;
    gltf.add_geometry(iData);
    gltf.finalise(relative_to_center, centerpoint);

    // Save it to a file
    fs::path fname = fs::path(manager.substitute_globals(filepath_));
    fs::create_directories(fname.parent_path());
    tinygltf::TinyGLTF t;
    if(!t.WriteGltfSceneToFile(&gltf.model, fname,
                            embed_images_, // embedImages
                            embed_buffers_, // embedBuffers
                            pretty_print_, // pretty print
                            binary_) // write binary
    ) {
      throw(gfIOError("Write GLTF failed"));
    }

  }
}