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
#include <bitset>

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"
#include <nlohmann/json.hpp>
#include <meshoptimizer.h>

namespace geoflow::nodes::basic3d
{
  typedef std::array<float,7> arr7f;
  typedef std::vector<char> vec1c;
  struct AttributeDataHelper {

    std::unordered_map<std::string, std::vector<arr7f>> data; // position [3f], normal[3f], feature_id[1f]
    std::unordered_map<std::string, unsigned> ftype_counts;

    arr3f center_point;
    NodeManager& manager;
    bool relative_to_center;

    float feature_id_cnt = 0.0;
    size_t total_count;

    AttributeDataHelper(
      const arr3f& center_point,
      NodeManager& manager,
      bool& relative_to_center
    ) : center_point(center_point), manager(manager), relative_to_center(relative_to_center)
    {
    }

    size_t get_total_feature_count() {
      size_t total = 0;
      for (const auto& [ftype, cnt] : ftype_counts) {
        total += cnt;
      }
      return total;
    }

    void add_feature(
      const std::string& feature_type,
      const TriangleCollection& tc, 
      const vec3f& normals
    ) {
      
      // count the nr of vertices and indices stored for this tc
      // std::set<arr6f> vertex_set;
      // std::map<arr6f, unsigned> vertex_map;
      size_t i = 0, v_cntr = 0;
      for (auto &triangle : tc)
      {
        for (auto &p_ : triangle)
        {
          const auto& n_ = normals[i];
          // normal is 0 for a degenerate triangle. skip such points
          if(!std::isnormal(n_[0]) && !std::isnormal(n_[1]) && !std::isnormal(n_[2])) {
            break;
          } 
          i++;

          auto p = manager.coord_transform_rev(p_);
          // NB: narrowing double to float here, not ideal
          // reproject n_
          arr3f pn_{p_[0]+n_[0], p_[1]+n_[1], p_[2]+n_[2]};
          auto pn = manager.coord_transform_rev(pn_);
          arr3f n{float(pn[0]-p[0]), float(pn[1]-p[1]), float(pn[2]-p[2])};
          auto l = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);

          if(relative_to_center) {
            p[0] -= float(center_point[0]); 
            p[1] -= float(center_point[1]);
            p[2] -= float(center_point[2]);
          }

          data[feature_type].push_back({
            float(p[0]), 
            float(p[1]), 
            float(p[2]), 
            n[0]/l, 
            n[1]/l, 
            n[2]/l,
            feature_id_cnt
          });
          ++total_count;
        }
      }
      feature_id_cnt += 1.0;
      if (ftype_counts.count(feature_type)) {
        ftype_counts[feature_type] += 1;
      } else {
        ftype_counts[feature_type] = 1;
      }
    }
  };

  struct MetadataHelper {
    // We need to get the attribute values from the input and create contiguous
    //  arrays from them, so that they can be added as a tightly-packed array
    //  to the buffer.
    struct StringAttributeOffset {
      unsigned current_offset{};
      std::vector<unsigned> offsets;
    };
    typedef std::variant<vec1b, vec1i, vec1f, vec1c> FlexVec;
    std::map<std::string, FlexVec> feature_attribute_map{};
    std::map<std::string, StringAttributeOffset> feature_attribute_string_offsets{};


    void add_metadata(gfMultiFeatureInputTerminal& attributes_inp, gfSingleFeatureInputTerminal& triangle_collections_inp) {
      auto fsize = triangle_collections_inp.size();
      for (auto& term : attributes_inp.sub_terminals()) {
        std::cout << "a count="<<term->size()<<std::endl;
        const auto& tname = term->get_name();
        if (term->accepts_type(typeid(bool))) {
          feature_attribute_map[tname] = std::move(vec1b{});
          std::get<vec1b>(feature_attribute_map[tname]).reserve(fsize);
        } else if (term->accepts_type(typeid(int))) {
          feature_attribute_map[tname] = std::move(vec1i{});
          std::get<vec1i>(feature_attribute_map[tname]).reserve(fsize);
        } else if (term->accepts_type(typeid(float))) {
          feature_attribute_map[tname] = std::move(vec1f{});
          std::get<vec1f>(feature_attribute_map[tname]).reserve(fsize);
        } else if (term->accepts_type(typeid(std::string))) {
          feature_attribute_map[tname] = std::move(vec1c{});
          std::get<vec1c>(feature_attribute_map[tname]).reserve(fsize*10);
          StringAttributeOffset soff;
          soff.current_offset = 0;
          soff.offsets = {0};
          feature_attribute_string_offsets.insert({tname, soff});
        }
      }

      for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
        if (!triangle_collections_inp.get_data_vec()[i].has_value())
          continue;
        const auto tc = triangle_collections_inp.get<TriangleCollection>(i);
        if (tc.vertex_count() == 0)
          continue;
        
        // WARNING: Cesium cannot handle 'unsigned long' and if you use it for data,
        //  the data just won't show up in the viewer, without giving any warnings.
        //  See the ComponentDatatype https://github.com/CesiumGS/cesium/blob/4855df37ee77be69923d6e57f807ca5b6219ad95/packages/engine/Source/Core/ComponentDatatype.js#L12
        //  It doesn't matter what their specs say, https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#component-type, https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#scalars, https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#strings
        //  that's a lie.
        for (auto& term : attributes_inp.sub_terminals()) {
          // if (!term->get_data_vec()[i].has_value()) {
          //   continue;
          // }
          const auto& tname = term->get_name();
          bool has_value = term->get_data_vec()[i].has_value();
          if (term->accepts_type(typeid(bool))) {
            if (has_value) {
              std::get<vec1b>(feature_attribute_map[tname]).emplace_back(term->get<const bool&>(i));
            } else {
              // NB bool cannot have proper nodata value, see: https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#required-properties-no-data-values-and-default-values
              std::get<vec1b>(feature_attribute_map[tname]).emplace_back(false);
            }
          } else if (term->accepts_type(typeid(int))) {
            if (has_value) {
              std::get<vec1i>(feature_attribute_map[tname]).emplace_back(term->get<const int&>(i));
            } else {
              std::get<vec1i>(feature_attribute_map[tname]).emplace_back(std::numeric_limits<int>::max());
            }
          } else if (term->accepts_type(typeid(float))) {
            if (has_value) {
              std::get<vec1f>(feature_attribute_map[tname]).emplace_back(term->get<const float&>(i));
            } else {
              std::get<vec1f>(feature_attribute_map[tname]).emplace_back(std::numeric_limits<float>::max());
            }
          } else if (term->accepts_type(typeid(std::string))) {
            if (has_value) {
              std::string string_attr = term->get<const std::string&>(i);
              // We store the string attributes in a contiguous vector of char-s. This
              // makes it simpler to calculate its size and copy it to the buffer.
              for (char s : string_attr) {
                std::get<vec1c>(feature_attribute_map[tname]).emplace_back(s);
              }
              // For string attributes, we also need to keep track of the offsets.
              auto string_byteLength = string_attr.size() * sizeof(char);
              feature_attribute_string_offsets[tname].current_offset += string_byteLength;
              feature_attribute_string_offsets[tname].offsets.emplace_back(feature_attribute_string_offsets[tname].current_offset);
            } else {
              // just push empty string for nodata
              feature_attribute_string_offsets[tname].offsets.emplace_back(feature_attribute_string_offsets[tname].current_offset);
            }
          } else {
            std::cout << "feature attribute " << tname << " is not bool/int/float/string" << std::endl;
          }
        }
      }
      // remove string attributes if there are not characters to write. This prevents potentially empty bufferviews later on.
      std::vector<std::string> to_erase;
      for (auto& [name, values] : feature_attribute_map) {
        if(const auto* vec = std::get_if<vec1c>(&values)){
          if (vec->size()==0) {
            to_erase.emplace_back(name);
          }
        }
      }
      for (auto& name : to_erase) {
        feature_attribute_map.erase(name);
        feature_attribute_string_offsets.erase(name);
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

  std::pair<arr7f,arr7f> get_min_max(std::vector<arr7f> values) {
    auto fmin = std::numeric_limits<float>::lowest();
    auto fmax = std::numeric_limits<float>::max();

    arr7f min {fmax, fmax, fmax, fmax, fmax, fmax, fmax};
    arr7f max {fmin, fmin, fmin, fmin, fmin, fmin, fmin};

    for (auto& v : values) {
      for(int i=0; i<7; ++i) {
        min[i] = std::min(min[i], v[i]);
        max[i] = std::max(max[i], v[i]);
      }
    }
    return {min, max};
  }

  struct BufferManager {
    tinygltf::Buffer buffer;
    size_t idx = 0;
    // stores the offset and length of the last appended chunk
    size_t byteOffset, byteLength;

    size_t sizeof_position = 3*sizeof(float);
    size_t sizeof_normal = sizeof(unsigned int);
    size_t sizeof_fid = sizeof(float);
    size_t vertex_byteSize = sizeof_position + sizeof_normal + sizeof_fid;

    void add_padding(size_t alignBy=4) {
      byteOffset = buffer.data.size();
      size_t padding = 0;
      padding = (alignBy-(byteOffset % alignBy)) % alignBy;
      for (int i=0; i<padding; ++i) {
        buffer.data.push_back('\0');
      }
      byteOffset += padding;
    }

    void append(unsigned char* src, const size_t& element_byteSize, const size_t& element_count, size_t alignBy=4) {
      // add padding if needed
      add_padding(alignBy);
      byteLength = element_count * element_byteSize;
      buffer.data.resize( buffer.data.size() + byteLength );
      memcpy(
        buffer.data.data() + byteOffset,
        src,
        byteLength
      );
    }

    void appendVerticesQuantized(const std::vector<arr7f>& vertices, size_t alignBy=4) {
      add_padding(alignBy);
      size_t element_count = vertices.size();
      buffer.data.resize(buffer.data.size() + element_count * vertex_byteSize);
      for (size_t i=0; i<element_count; ++i) {
        memcpy(
          buffer.data.data() + byteOffset + i*vertex_byteSize,
          (unsigned char*)&vertices[i][0],
          sizeof_position
        );
        // quantize normals
        auto nx = (int8_t)(meshopt_quantizeSnorm(vertices[i][3], 8));
        auto ny = (int8_t)(meshopt_quantizeSnorm(vertices[i][4], 8));
        auto nz = (int8_t)(meshopt_quantizeSnorm(vertices[i][5], 8));
        buffer.data[byteOffset + i*vertex_byteSize + sizeof_position+0] = *(unsigned char*)&nx;
        buffer.data[byteOffset + i*vertex_byteSize + sizeof_position+1] = *(unsigned char*)&ny;
        buffer.data[byteOffset + i*vertex_byteSize + sizeof_position+2] = *(unsigned char*)&nz;
        buffer.data[byteOffset + i*vertex_byteSize + sizeof_position+3] = *(unsigned char*)&"\0";
        memcpy(
          buffer.data.data() + byteOffset + i*vertex_byteSize + sizeof_position + sizeof_normal,
          (unsigned char*)&vertices[i][6],
          sizeof_fid
        );
      }
      byteLength = element_count * vertex_byteSize;
    }
  };

  struct GLTFBuilder {

    tinygltf::Model model;
    tinygltf::Mesh mesh;
    BufferManager buffer;
    
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

    std::vector<double> hex2rgb(int hexValue) {
      return {
        ((hexValue >> 16) & 0xFF) / 255.0,  // Extract the RR byte
        ((hexValue >> 8) & 0xFF) / 255.0,   // Extract the GG byte
        ((hexValue) & 0xFF) / 255.0,         // Extract the BB byte
        1.0
      };
    }

    size_t create_material(std::string type) {
      //building red
      if (type == "Building" or type == "BuildingPart" or type == "BuildingInstallation") {
        tinygltf::Material building;
        building.pbrMetallicRoughness.baseColorFactor = hex2rgb(0xEC7658);
        model.materials.push_back(building);
      }
      // terrain brown
      else if (type == "TINRelief") {
        tinygltf::Material terrain;
        terrain.pbrMetallicRoughness.baseColorFactor = hex2rgb(0xA6CD59);
        model.materials.push_back(terrain);
      }
      // transport grey
      else if (type == "Road" or type == "Railway" or type == "TransportSquare") {
        tinygltf::Material  transport;
        transport.pbrMetallicRoughness.baseColorFactor = hex2rgb(0x474447);
        model.materials.push_back(transport);
      }
      // waterbody blue
      else if (type == "WaterBody") {
        tinygltf::Material waterbody;
        waterbody.pbrMetallicRoughness.baseColorFactor = hex2rgb(0x293A4A);
        model.materials.push_back(waterbody);
      }
      // vegetation green
      else if (type == "PlantCover" or type == "SolitaryVegetationObject") {
        tinygltf::Material vegetation;
        vegetation.pbrMetallicRoughness.baseColorFactor = hex2rgb(0xA6CD59);
        model.materials.push_back(vegetation);
      }
      // landuse yellow
      else if (type == "LandUse") {
        tinygltf::Material  landuse;
        landuse.pbrMetallicRoughness.baseColorFactor = hex2rgb(0xC3DBB5);
        model.materials.push_back(landuse);
      }
      // CityFurniture orange
      else if (type == "CityFurniture") {
        tinygltf::Material  CityFurniture;
        CityFurniture.pbrMetallicRoughness.baseColorFactor = hex2rgb(0x4F4A6A);
        model.materials.push_back(CityFurniture);
      }
      // bridge purple
      else if (type == "Bridge" or type == "BridgePart" or type == "BridgeInstallation" or type == "BridgeConstructionElement") {
        tinygltf::Material bridge;
        bridge.pbrMetallicRoughness.baseColorFactor = hex2rgb(0x4F4A6A);
        model.materials.push_back(bridge);
      }
      // tunnel black
      else if (type == "Tunnel" or type == "TunnelPart" or type == "TunnelInstallation") {
        tinygltf::Material tunnel;
        tunnel.pbrMetallicRoughness.baseColorFactor = hex2rgb(0x4F4A6A);
        model.materials.push_back(tunnel);
      }
      // GenericCityObject pink
      else if (type == "GenericCityObject") {
        tinygltf::Material GenericCityObject;
        GenericCityObject.pbrMetallicRoughness.baseColorFactor = hex2rgb(0x4F4A6A);
        model.materials.push_back(GenericCityObject);
      }
      return model.materials.size()-1;
    }

    void add_geometry(AttributeDataHelper& IDH, bool quantizeVertex) {
      size_t feature_id_set_idx = 0;
      for (auto& [ftype, data] : IDH.data) {
        tinygltf::Primitive  primitive;
        tinygltf::BufferView bf_indices;
        tinygltf::BufferView bf_attributes;
        tinygltf::Accessor   acc_positions;
        tinygltf::Accessor   acc_normals;
        tinygltf::Accessor   acc_indices;
        tinygltf::Accessor   acc_feature_ids;

        // create indices and remap vertices
        size_t index_count = data.size();
        std::vector<unsigned int> remap(index_count); // allocate temporary memory for the remap table
        size_t vertex_count = meshopt_generateVertexRemap(&remap[0], NULL, index_count, &data[0], index_count, sizeof(arr7f));

        std::vector<unsigned> indices(index_count);
        std::vector<arr7f> vertices(vertex_count);
        meshopt_remapIndexBuffer(&indices[0], NULL, index_count, &remap[0]);
        meshopt_remapVertexBuffer(&vertices[0], &data[0], index_count, sizeof(arr7f), &remap[0]);
        meshopt_optimizeVertexCache(&indices[0], &indices[0], index_count, vertex_count);
        meshopt_optimizeOverdraw(&indices[0], &indices[0], index_count, &vertices[0][0], vertex_count, sizeof(arr7f), 1.05f);
        meshopt_optimizeVertexFetch(&vertices[0], &indices[0], index_count, &vertices[0], vertex_count, sizeof(arr7f));

        // indices copy data
        auto [min, max] = std::minmax_element(begin(indices), end(indices));
        size_t element_byteSize = sizeof(unsigned);
        if ((*max <= std::numeric_limits<unsigned short>::max())) {
          element_byteSize = sizeof(unsigned short);
          std::vector<unsigned short> part( indices.begin(), indices.end() );
          buffer.append((unsigned char*)part.data(), element_byteSize, index_count);
        } else { //if (max <= numeric_limits<unsigned int>::max()) {
          buffer.append((unsigned char*)indices.data(), element_byteSize, index_count);
        }

        // indices setup bufferview
        bf_indices.buffer     = buffer.idx;
        bf_indices.byteOffset = buffer.byteOffset;
        bf_indices.byteLength = buffer.byteLength;
        bf_indices.target     = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
        auto id_bf_indices    = model.bufferViews.size();
        model.bufferViews.push_back(bf_indices);
        
        // indices setup accessor
        acc_indices.bufferView    = id_bf_indices;
        // acc_indices.byteOffset    = 0;
        acc_indices.type          = TINYGLTF_TYPE_SCALAR;
        if (element_byteSize == sizeof(unsigned short)) {
          acc_indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT;
        } else {
          acc_indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
        }
        acc_indices.count         = index_count;
        acc_indices.minValues     = { double(*min) };
        acc_indices.maxValues     = { double(*max) };
        primitive.indices = model.accessors.size();
        model.accessors.push_back(acc_indices);
        
        // attributes copy data
        auto [amin, amax] = get_min_max(vertices);
        size_t sizeof_position = 3*sizeof(float);
        size_t sizeof_normal = 3*sizeof(float);
        size_t sizeof_fid = sizeof(float);
        element_byteSize = sizeof_position + sizeof_normal + sizeof_fid;

        if (quantizeVertex){
          buffer.appendVerticesQuantized(vertices);
          sizeof_position = buffer.sizeof_position;
          sizeof_normal = buffer.sizeof_normal;
          sizeof_fid = buffer.sizeof_fid;
          element_byteSize = buffer.vertex_byteSize;
        } else {
          buffer.append((unsigned char*)vertices.data(), element_byteSize, vertex_count);
        }

        // attributes setup bufferview
        bf_attributes.buffer     = buffer.idx;
        bf_attributes.byteOffset = buffer.byteOffset;
        bf_attributes.byteStride = element_byteSize;
        bf_attributes.byteLength = buffer.byteLength;
        bf_attributes.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
        auto id_bf_attributes    = model.bufferViews.size();
        model.bufferViews.push_back(bf_attributes);

        // positions accessor
        acc_positions.bufferView    = id_bf_attributes;
        acc_positions.byteOffset    = 0;
        acc_positions.type          = TINYGLTF_TYPE_VEC3;
        acc_positions.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        acc_positions.count         = vertex_count;
        acc_positions.minValues = { amin[0], amin[1], amin[2] };
        acc_positions.maxValues = { amax[0], amax[1], amax[2] };
        primitive.attributes["POSITION"] = model.accessors.size(); // The index of the accessor for positions
        model.accessors.push_back(acc_positions);
        
        // normals accessor
        acc_normals.bufferView    = id_bf_attributes;
        acc_normals.byteOffset    = sizeof_position;
        acc_normals.count         = vertex_count;
        acc_normals.type          = TINYGLTF_TYPE_VEC3;
        if (quantizeVertex){
          acc_normals.componentType = TINYGLTF_COMPONENT_TYPE_BYTE;
          acc_normals.normalized = true;
          acc_normals.minValues = { (double)meshopt_quantizeSnorm(float(amin[3]),8), (double)meshopt_quantizeSnorm(float(amin[4]),8), (double)meshopt_quantizeSnorm(float(amin[5]),8) };
          acc_normals.maxValues = { (double)meshopt_quantizeSnorm(float(amax[3]),8), (double)meshopt_quantizeSnorm(float(amax[4]),8), (double)meshopt_quantizeSnorm(float(amax[5]),8) };
        } else {
          acc_normals.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
          acc_normals.minValues = { amin[3], amin[4], amin[5] };
          acc_normals.maxValues = { amax[3], amax[4], amax[5] };
        }
        primitive.attributes["NORMAL"] = model.accessors.size(); // The index of the accessor for normals
        model.accessors.push_back(acc_normals);
        
        // feature_ids accessor
        acc_feature_ids.bufferView    = id_bf_attributes;
        acc_feature_ids.byteOffset    = sizeof_position + sizeof_normal;
        acc_feature_ids.type          = TINYGLTF_TYPE_SCALAR;
        acc_feature_ids.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        acc_feature_ids.count         = vertex_count;
        acc_feature_ids.minValues = { amin[6] };
        acc_feature_ids.maxValues = { amax[6] };
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
      model.extensionsUsed.emplace_back("EXT_mesh_features");
    }

    void add_metadata(MetadataHelper& MH, std::string metadata_class_name, size_t total_feature_count) {
      // Schema definition
      tinygltf::Value::Object metadata_class_properties;
      for (const auto& [name, value_vec] : MH.feature_attribute_map) {
        tinygltf::Value::Object metadata_property;
        // metadata_property["description"] = tinygltf::Value((std::string)"property description");
        if (std::holds_alternative<vec1b>(value_vec)) {
          metadata_property["type"] = tinygltf::Value((std::string)"BOOLEAN");
        } else if (std::holds_alternative<vec1i>(value_vec)) {
          metadata_property["type"] = tinygltf::Value((std::string)"SCALAR");
          metadata_property["componentType"] = tinygltf::Value((std::string)"INT32");
          metadata_property["noData"] = tinygltf::Value(std::numeric_limits<int>::max());
        } else if (std::holds_alternative<vec1f>(value_vec)) {
          metadata_property["type"] = tinygltf::Value((std::string)"SCALAR");
          metadata_property["componentType"] = tinygltf::Value((std::string)"FLOAT32");
          metadata_property["noData"] = tinygltf::Value(std::numeric_limits<float>::max());
        } else if (std::holds_alternative<vec1c>(value_vec)) {
          metadata_property["type"] = tinygltf::Value((std::string)"STRING");
        } else {
          std::cout << "unhandled feature attribute type" << std::endl;
        }
        metadata_class_properties[name] = tinygltf::Value(metadata_property);
      }
      tinygltf::Value::Object metadata_class;
      metadata_class["name"] = tinygltf::Value(metadata_class_name);
      metadata_class["description"] = tinygltf::Value((std::string)"class description");
      metadata_class["properties"] = tinygltf::Value(metadata_class_properties);
      tinygltf::Value::Object metadata_classes;
      metadata_classes[metadata_class_name] = tinygltf::Value(metadata_class);
      tinygltf::Value::Object metadata_schema;
      metadata_schema["classes"] = tinygltf::Value(metadata_classes);

      // Add the property tables for the feature attributes. The property tables
      // reference bufferviews, which reference the complete array of the values
      // of one feature attribute (column-based layout).
      // https://github.com/CesiumGS/glTF/tree/3d-tiles-next/extensions/2.0/Vendor/EXT_structural_metadata#ext_structural_metadata
      // We need a bufferView for each feature attribute, plus the string offsets
      // in case of string attributes.
      std::map<std::string, std::tuple<unsigned long, unsigned long>> id_bf_attr_map{};
      unsigned long byteLength(0);
      for (auto& [name, value_vec] : MH.feature_attribute_map) {
        // bufferView ID for the attribute
        unsigned long id_bf_attr;
        // bufferView ID for the attribute offsets https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#strings
        unsigned long id_bf_attr_offset(0); // this is always >0 if used
        {
          tinygltf::BufferView bf_attr;
          if (const vec1b* vec = std::get_if<vec1b>(&value_vec)) {
            std::cout << "bool attribute not stored!!\n";
            // see https://stackoverflow.com/questions/46115669/why-does-stdvectorbool-have-no-data
            // buffer.append((unsigned char*)(&vec->operator[](0)), vec->size(), sizeof(bool), 8);
          } else if (const vec1i* vec = std::get_if<vec1i>(&value_vec)) {
            buffer.append((unsigned char*)vec->data(), vec->size(), sizeof(int), 8);
          } else if (const vec1f* vec = std::get_if<vec1f>(&value_vec)) {
            buffer.append((unsigned char*)vec->data(), vec->size(), sizeof(float), 8);
          } else if (const vec1c* vec = std::get_if<vec1c>(&value_vec)) {
            buffer.append((unsigned char*)vec->data(), vec->size(), sizeof(char), 8);
          }
          // Everything goes into the same buffer
          bf_attr.buffer     = buffer.idx;
          bf_attr.byteOffset = buffer.byteOffset;
          bf_attr.byteLength = buffer.byteLength;
          bf_attr.byteStride = 0; // all feature attribute value arrays need to be tightly packed https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#binary-table-format
          bf_attr.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
          id_bf_attr = model.bufferViews.size();
          model.bufferViews.push_back(bf_attr);
        }
        if (std::holds_alternative<vec1c>(value_vec)) {
          tinygltf::BufferView bf_attr;
          auto& value_vec_offs = MH.feature_attribute_string_offsets[name].offsets;
          buffer.append((unsigned char*)value_vec_offs.data(), value_vec_offs.size(), sizeof(unsigned), 8);
          // Everything goes into the same buffer
          bf_attr.buffer     = buffer.idx;
          bf_attr.byteOffset = buffer.byteOffset;
          bf_attr.byteLength = buffer.byteLength;
          bf_attr.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
          id_bf_attr_offset = model.bufferViews.size();
          model.bufferViews.push_back(bf_attr);
        }
        id_bf_attr_map[name] = {id_bf_attr, id_bf_attr_offset};
      }

      tinygltf::Value::Object properties_property_tbl;
      for (const auto& [name, buffer_ids] : id_bf_attr_map) {
        tinygltf::Value::Object property_tbl_attr;
        auto                    buffer_id         = std::get<0>(buffer_ids);
        auto                    buffer_id_offsets = std::get<1>(buffer_ids);
        if (buffer_id_offsets == 0) {
          property_tbl_attr["values"] = tinygltf::Value((int)buffer_id);
        } else {
          property_tbl_attr["values"] = tinygltf::Value((int)buffer_id);
          property_tbl_attr["stringOffsets"] =
            tinygltf::Value((int)buffer_id_offsets);
          property_tbl_attr["stringOffsetType"] =
            tinygltf::Value((std::string) "UINT32");
        }
        properties_property_tbl[name] = tinygltf::Value(property_tbl_attr);
      }
      tinygltf::Value::Object property_table;
      property_table["class"] = tinygltf::Value((std::string)metadata_class_name);
      property_table["count"] = tinygltf::Value((int)total_feature_count); // feature count
      property_table["name"] = tinygltf::Value((std::string)"propertyTable name");
      property_table["properties"] = tinygltf::Value(properties_property_tbl);

      tinygltf::Value::Object ext_structural_metadata_object;
      ext_structural_metadata_object["schema"] = tinygltf::Value(metadata_schema);
      tinygltf::Value::Array property_tables;
      property_tables.emplace_back(property_table);
      ext_structural_metadata_object["propertyTables"] = tinygltf::Value(property_tables);
      tinygltf::ExtensionMap root_extensions;
      root_extensions["EXT_structural_metadata"] = tinygltf::Value(ext_structural_metadata_object);
      model.extensions = root_extensions;
      model.extensionsUsed.emplace_back("EXT_structural_metadata");
    }

    void finalise(bool relative_to_center, const arr3f centerpoint) {

      // add buffer and mesh objects to model
      model.buffers.push_back(buffer.buffer);
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

      model.extensionsUsed.emplace_back("KHR_mesh_quantization");
      model.extensionsRequired.emplace_back("KHR_mesh_quantization");
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
    std::cout << "tc count="<<triangle_collections_inp.size()<<std::endl;
    std::cout << "a count="<<triangle_collections_inp.size()<<std::endl;
    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value()) {
        std::cout << "skip tc i="<<i<<std::endl;
        continue;
      }
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0) {
        std::cout << "skip tc i="<<i<<std::endl;
        continue;
      }
      global_bbox.add(manager.coord_transform_rev(tc[0][1]));
    }
    arr3f centerpoint = global_bbox.center();

    // create intermediate vectors
    AttributeDataHelper iData(centerpoint, manager, relative_to_center);

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

    // attributes
    MetadataHelper mData;
    mData.add_metadata(attributes_inp, triangle_collections_inp);

    // build the gltf
    GLTFBuilder gltf;
    gltf.add_geometry(iData, quantizeVertex);
    gltf.add_metadata(mData, manager.substitute_globals(metadata_class_name_), iData.get_total_feature_count());
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