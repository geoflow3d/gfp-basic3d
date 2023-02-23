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
#include <nlohmann/json.hpp>

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

  unsigned get_material_id(const std::string type) {
    // std::cout << "assigning material for " << type << std::endl;
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
    std::cout << std::endl;
    typedef std::array<float,6> arr6f;
    struct TCInfo {
      size_t i_input;
      unsigned vertex_offset;
      unsigned vertex_count;
      unsigned index_offset;
      unsigned index_count;
      unsigned feature_id_count;
      float    feature_id;
      std::vector<double> index_min;
      std::vector<double> index_max;
      std::vector<double> normals_min;
      std::vector<double> positions_min;
      std::vector<double> normals_max;
      std::vector<double> positions_max;
    };
    struct StringAttributeOffset {
      unsigned current_offset{};
      std::vector<unsigned> offsets;
    };

    auto& triangle_collections_inp = vector_input("triangles");
    auto& normals_inp = vector_input("normals");
    auto& featuretype_inp = vector_input("feature_type");
    auto& attributes_inp = poly_input("attributes");

    if (metadata_class_name_.empty()) {
      throw(gfException("metadata_class must be provided"));
    }

    tinygltf::Model model;
    model.asset.generator = "Geoflow";
    model.asset.version = "2.0";
    tinygltf::Scene scene;
    tinygltf::Node node;
    tinygltf::Mesh mesh;
    tinygltf::Buffer buffer;

    // We need to get the attribute values from the input and create contiguous
    //  arrays from them, so that they can be added as a tightly-packed array
    //  to the buffer.
    std::map<std::string, std::vector<std::any>> feature_attribute_map{};
    std::map<std::string, StringAttributeOffset> feature_attribute_string_offsets{};

    // Create a simple material
    // tinygltf::Material mat;
    // mat.pbrMetallicRoughness.baseColorFactor = {1.0f, 0.9f, 0.9f, 1.0f};  
    // mat.doubleSided = true;
    // model.materials.push_back(mat);
    create_materials(model);

    // BEGIN METADATA

    // WARNING: Using anything else than a 0-based, sequential ID as
    //  _FEATURE_ID, together with feature silently breaks cesium.
    //  The features are rendered and
    //  they show up, but there is something wrong with them, because they are
    //  not recognized as 3dtilefeatures when trying to pick them on the screen.
    //  Such that Cesium.viewer.scene.pick(movement.endPosition); returns an
    //  'undefined' when picking a 3d tile feature. So for when Cesium is used
    //  for rendering the 3D Tiles, only do sequential featureids.
    //  This is because Cesium (still, in v1.102) only implement the 3D Tiles
    //  v1.0 specs when it comes to feture IDs: https://github.com/CesiumGS/3d-tiles/blob/main/specification/TileFormats/Batched3DModel/README.md#batch-table
    //  batchID (v1.0) == featureID (v1.1)
    // Ideally, we want to use the 'objectid' as _FEATURE_ID, because
    //  otherwise it'll be cumbersome to get the metadata from an API for the
    //  selected feature I think.
    //  https://github.com/CesiumGS/glTF/tree/3d-tiles-next/extensions/2.0/Vendor/EXT_mesh_features#referencing-external-resources-with-feature-ids
    // Check if 'feature_id' is provided, and the attribute value of the first
    // feature can be cast to a float. If not true, then use sequential ID.
    // It is ok to only check the first feature, because each subequent feature
    // is supposed to have the same type, since they have been processed already
    // put into a vector.
    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      if (!feature_id_attribute_.empty()) {
        std::cout << "feature_id was provided, checking for type compatibility" << std::endl;
        for (auto& term : attributes_inp.sub_terminals()) {
          if (!term->get_data_vec()[i].has_value())
            continue;
          const auto& tname = term->get_name();
          if (tname == feature_id_attribute_) {
            if (term->accepts_type(typeid(bool))) {
              std::cout << "the type of the value of " << tname << " in the first feature is boolean and it cannot be reliably cast to a float. Defaulting to sequential _FEATURE_ID-s." << std::endl;
              feature_id_attribute_ = "";
            } else if (term->accepts_type(typeid(std::string))) {
              std::cout << "the type of the value of " << tname << " in the first feature is string and it cannot be reliably cast to a float. Defaulting to sequential _FEATURE_ID-s." << std::endl;
              feature_id_attribute_ = "";
            }
          }
        }
      }
      break;
    }

    // Init the feature attribute container
    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      std::cout << "initializing attribute container" << std::endl;
      for (auto& term : attributes_inp.sub_terminals()) {
        if (!term->get_data_vec()[i].has_value())
          continue;
        const auto& tname = term->get_name();
        feature_attribute_map.insert({tname, std::vector<std::any>()});
        if (term->accepts_type(typeid(std::string))) {
          StringAttributeOffset soff;
          soff.current_offset = 0;
          soff.offsets = {0};
          feature_attribute_string_offsets.insert({tname, soff});
        }
      }
      break;
    }

    // END METADATA

    // BEGIN CRS handling and finding centerpoint

    manager.set_rev_crs_transform(manager.substitute_globals(CRS_).c_str());

    // determine approximate centerpoint
    // if(relative_to_center) {
    Box global_bbox;
    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0)
        continue;
      global_bbox.add(manager.coord_transform_rev(tc[0][1]));
    }
    arr3f c = global_bbox.center();
    // }

    // BEGIN INIT INTERMEDIATE VECTORS

    int feature_id_set_idx = 0;
    // WARNING: Cesium (v1.102) only reads the first buffer from gltf.buffers.
    int buffer_idx = 0;

    std::vector<arr6f> vertex_vec; // position + normal
    std::vector<float> feature_id_vec; // feature id vertex attribute
    std::vector<unsigned> index_vec;
    std::vector<TCInfo>   info_vec;
    unsigned v_offset = 0;
    unsigned i_offset = 0;

    // It's a float, because glTF accessors do not support UNSIGNED_INT types
    // for 32-bit integers in vertex attributes. There is UNSIGNED_SHORT, but
    // probably there could be more than ~65k individual features that we need
    // to identify in one gltf file.
    float feature_id = 0.0;
    float feature_id_seq = 0.0;

    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0)
        continue;
      auto& normals = normals_inp.get<vec3f>(i);
      
      if (feature_id_attribute_.empty()) {
        feature_id = feature_id_seq;
      } else {
        // Using a feature attribute value as the _FEATURE_ID vertex attribute
        for (auto& term : attributes_inp.sub_terminals()) {
          if (!term->get_data_vec()[i].has_value())
            continue;
          auto tname = term->get_name();
          if (tname == feature_id_attribute_) {
            if (term->accepts_type(typeid(float))) {
              feature_id = term->get<const float&>(i);
            } else if (term->accepts_type(typeid(int))) {
              feature_id = (float)term->get<const int&>(i);
            } else {
              std::cout << "the type of " << term << " is not int or float for value " << term->get<const std::string&>(i) << " and it cannot be used as a _FEATURE_ID vertex attribute, which requires a float type. Defaulting to the sequence value for this feature's _FEATURE_ID." << std::endl;
              feature_id = feature_id_seq;
            }
          }
        }
      }

      TCInfo inf;
      inf.i_input = i;
      Box positions_box;
      // compute arrays
      {
        // count the nr of vertices and indices stored for this tc
        unsigned v_cntr = 0, i = 0, fid_cntr = 0;
        std::set<arr6f> vertex_set;
        std::map<arr6f, unsigned> vertex_map;
        for (auto &triangle : tc)
        {
          for (auto &p : triangle)
          {
            const auto p_ = manager.coord_transform_rev(p);
            // NB: narrowing double to float here, not ideal
            arr6f vertex;
            if(relative_to_center) {
              vertex = arr6f{
                float(p_[0] - c[0]), 
                float(p_[1] - c[1]), 
                float(p_[2] - c[2]), 
                normals[i][0], normals[i][1], normals[i][2]
              };
            } else {
              vertex = arr6f{
                float(p_[0]), 
                float(p_[1]), 
                float(p_[2]), 
                normals[i][0], normals[i][1], normals[i][2]
              };
            }
            auto [it, did_insert] = vertex_set.insert(vertex);
            if (did_insert)
            {
              vertex_map[vertex] = v_cntr++;
              
              // reproject normal
              arr3f pn_{p[0]+vertex[3], p[1]+vertex[4], p[2]+vertex[5]};
              auto pn = manager.coord_transform_rev(pn_);
              arr3f n{float(pn[0]-p_[0]), float(pn[1]-p_[1]), float(pn[2]-p_[2])};
              auto l = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
              vertex_vec.push_back(vertex);
              vertex_vec.back()[3] = n[0]/l;
              vertex_vec.back()[4] = n[1]/l;
              vertex_vec.back()[5] = n[2]/l;
              feature_id_vec.push_back(feature_id);
              fid_cntr++;
              positions_box.add(arr3f{vertex[0], vertex[1], vertex[2]});
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
        inf.feature_id_count = fid_cntr;
        inf.feature_id = feature_id;

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
      feature_id_seq += 1.0;
      // WARNING: Cesium cannot handle 'unsigned long' and if you use it for data,
      //  the data just won't show up in the viewer, without giving any warnings.
      //  See the ComponentDatatype https://github.com/CesiumGS/cesium/blob/4855df37ee77be69923d6e57f807ca5b6219ad95/packages/engine/Source/Core/ComponentDatatype.js#L12
      //  It doesn't matter what their specs say, https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#component-type, https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#scalars, https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#strings
      //  that's a lie.
      for (auto& term : attributes_inp.sub_terminals()) {
        if (!term->get_data_vec()[i].has_value()) {
          continue;
        }
        const auto& tname = term->get_name();
        if (term->accepts_type(typeid(bool))) {
          feature_attribute_map[tname].emplace_back(term->get<const bool&>(i));
        } else if (term->accepts_type(typeid(int))) {
          feature_attribute_map[tname].emplace_back(term->get<const int&>(i));
        } else if (term->accepts_type(typeid(float))) {
          feature_attribute_map[tname].emplace_back(term->get<const float&>(i));
        } else if (term->accepts_type(typeid(std::string))) {
          std::string string_attr = term->get<const std::string&>(i);
          // We store the string attributes in a contiguous vector of char-s. This
          // makes it simpler to calculate its size and copy it to the buffer.
          for (char s : string_attr) {
            feature_attribute_map[tname].emplace_back(s);
          }
          // For string attributes, we also need to keep track of the offsets.
          auto string_byteLength = string_attr.size() * sizeof(char);
          feature_attribute_string_offsets[tname].current_offset += string_byteLength;
          feature_attribute_string_offsets[tname].offsets.emplace_back(feature_attribute_string_offsets[tname].current_offset);
        } else {
          std::cout << "feature attribute " << tname << " is not bool/int/float/string" << std::endl;
        }
      }

      info_vec.push_back(inf);
    }
    manager.clear_rev_crs_transform();

    // END INIT INTERMEDIATE VECTORS

    if (info_vec.size() == 0) {
      std::cout<<"no vertices to write, aborting...\n";
      return;
    }
    
    // TODO check if padding is needed -> no only if componentype of index and attribute has different byteSize
    auto byteOffset_attributes = index_vec.size() * sizeof(unsigned);
    auto byteOffset_feature_id = index_vec.size()*sizeof(unsigned) + vertex_vec.size()*6*sizeof(float);

    // Add the Mesh primitives
    for (unsigned i = 0; i < info_vec.size(); ++i) {
      auto&                inf   = info_vec[i];
      auto&                ftype = featuretype_inp.get<std::string>(inf.i_input);
      tinygltf::BufferView bf_attributes;
      tinygltf::BufferView bf_indices;
      tinygltf::BufferView bf_feature_ids;
      tinygltf::Accessor   acc_positions;
      tinygltf::Accessor   acc_normals;
      tinygltf::Accessor   acc_indices;
      tinygltf::Accessor   acc_feature_ids;
      tinygltf::Primitive  primitive;

      bf_indices.buffer     = buffer_idx;
      bf_indices.byteOffset = inf.index_offset * sizeof(unsigned);
      bf_indices.byteLength = inf.index_count * sizeof(unsigned);
      bf_indices.target     = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
      auto id_bf_indices    = model.bufferViews.size();
      model.bufferViews.push_back(bf_indices);

      acc_indices.bufferView    = id_bf_indices;
      acc_indices.byteOffset    = 0;
      acc_indices.type          = TINYGLTF_TYPE_SCALAR;
      acc_indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
      acc_indices.count         = inf.index_count;
      acc_indices.minValues     = { inf.index_min };
      acc_indices.maxValues     = { inf.index_max };
      model.accessors.push_back(acc_indices);
      primitive.indices = model.accessors.size() - 1;

      bf_attributes.buffer = buffer_idx;
      bf_attributes.byteOffset =
        byteOffset_attributes + inf.vertex_offset * 6 * sizeof(float);
      bf_attributes.byteLength = inf.vertex_count * 6 * sizeof(float);
      bf_attributes.byteStride = 6 * sizeof(float);
      bf_attributes.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
      auto id_bf_attributes    = model.bufferViews.size();
      model.bufferViews.push_back(bf_attributes);

      acc_positions.bufferView    = id_bf_attributes;
      acc_positions.byteOffset    = 0;
      acc_positions.type          = TINYGLTF_TYPE_VEC3;
      acc_positions.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_positions.count         = inf.vertex_count;
      acc_positions.minValues     = inf.positions_min;
      acc_positions.maxValues     = inf.positions_max;
      model.accessors.push_back(acc_positions);
      primitive.attributes["POSITION"] =
        model.accessors.size() - 1; // The index of the accessor for positions

      acc_normals.bufferView    = id_bf_attributes;
      acc_normals.byteOffset    = 3 * sizeof(float);
      acc_normals.type          = TINYGLTF_TYPE_VEC3;
      acc_normals.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_normals.count         = inf.vertex_count;
      // FIXME: "Declared minimum value for this component (-0.7280862927436829) does not match actual minimum (-0.6278660297393799)."
      // FIXME: "Accessor element at index .. is NaN." This occurs occaisonally for some objects, for the NORMALS accesor
      // FIXME: handle degenerate triangles
//      acc_normals.minValues     = inf.normals_min;
//      acc_normals.maxValues     = inf.normals_max;
      model.accessors.push_back(acc_normals);
      primitive.attributes["NORMAL"] =
        model.accessors.size() - 1; // The index of the accessor for normals

      bf_feature_ids.buffer = buffer_idx;
      bf_feature_ids.byteOffset =
        byteOffset_feature_id + inf.vertex_offset * sizeof(float);
      bf_feature_ids.byteLength = inf.vertex_count * sizeof(float);
      bf_feature_ids.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
      auto id_bf_feature_ids    = model.bufferViews.size();
      model.bufferViews.push_back(bf_feature_ids);

      acc_feature_ids.bufferView       = id_bf_feature_ids;
      acc_feature_ids.type             = TINYGLTF_TYPE_SCALAR;
      acc_feature_ids.normalized       = false;
      acc_feature_ids.componentType    = TINYGLTF_COMPONENT_TYPE_FLOAT;
      acc_feature_ids.count            = inf.feature_id_count;
      acc_feature_ids.maxValues        = {inf.feature_id};
      acc_feature_ids.minValues        = {inf.feature_id};
      // This is an int, because tinygltf::Value only knows 'int' (no unsigned etc).
      int id_acc_feature_ids = model.accessors.size();
      model.accessors.push_back(acc_feature_ids);
      std::string fid = "_FEATURE_ID_" + std::to_string(feature_id_set_idx );
      primitive.attributes[fid] = id_acc_feature_ids;

      tinygltf::ExtensionMap primitive_extensions;
      primitive_extensions["EXT_mesh_features"] = create_ext_mesh_features(
        1, feature_id_set_idx, 0, feature_id_attribute_);

      primitive.material   = get_material_id(ftype);
      primitive.mode       = TINYGLTF_MODE_TRIANGLES;
      primitive.extensions = primitive_extensions;
      mesh.primitives.push_back(primitive);
    }

    // EXT_structural_metadata
    // Schema definition
    tinygltf::Value::Object metadata_class_properties;
    for (const auto& [name, value_vec] : feature_attribute_map) {
      tinygltf::Value::Object metadata_property;
      metadata_property["description"] = tinygltf::Value((std::string)"property description");
      if (value_vec[0].type() == typeid(bool)) {
        metadata_property["type"] = tinygltf::Value((std::string)"BOOLEAN");
      } else if (value_vec[0].type() == typeid(int)) {
        metadata_property["type"] = tinygltf::Value((std::string)"SCALAR");
        metadata_property["componentType"] = tinygltf::Value((std::string)"INT32");
      } else if (value_vec[0].type() == typeid(float)) {
        metadata_property["type"] = tinygltf::Value((std::string)"SCALAR");
        metadata_property["componentType"] = tinygltf::Value((std::string)"FLOAT32");
      } else if (value_vec[0].type() == typeid(char)) {
        metadata_property["type"] = tinygltf::Value((std::string)"STRING");
      } else {
        std::cout << "unhandled feature attribute type" << std::endl;
      }
      metadata_class_properties[name] = tinygltf::Value(metadata_property);
    }
    std::string metadata_class_name = metadata_class_name_;
    tinygltf::Value::Object metadata_class;
    metadata_class["name"] = tinygltf::Value(metadata_class_name_);
    metadata_class["description"] = tinygltf::Value((std::string)"class description");
    metadata_class["properties"] = tinygltf::Value(metadata_class_properties);
    tinygltf::Value::Object metadata_classes;
    metadata_classes[metadata_class_name_] = tinygltf::Value(metadata_class);
    tinygltf::Value::Object metadata_schema;
    metadata_schema["classes"] = tinygltf::Value(metadata_classes);

    // Add the property tables for the feature attributes. The property tables
    // reference bufferviews, which reference the complete array of the values
    // of one feature attribute (column-based layout).
    // https://github.com/CesiumGS/glTF/tree/3d-tiles-next/extensions/2.0/Vendor/EXT_structural_metadata#ext_structural_metadata
    // We need a bufferView for each feature attribute, plus the string offsets
    // in case of string attributes.
    std::map<std::string, std::tuple<unsigned long, unsigned long>> id_bf_attr_map{};
    auto byteOffset_feature_attributes =
        byteOffset_feature_id + feature_id_vec.size() * sizeof(float);
    unsigned long total_attr_vec_size(0);
    for (auto& [name, value_vec] : feature_attribute_map) {
      value_vec.shrink_to_fit();
      // bufferView ID for the attribute
      unsigned long id_bf_attr;
      // bufferView ID for the attribute offsets https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#strings
      unsigned long id_bf_attr_offset(0); // this is always >0 if used
      {
        tinygltf::BufferView bf_attr;
        unsigned long type_size(0);
        if (value_vec[0].type() == typeid(bool)) {
          type_size = sizeof(bool);
        } else if (value_vec[0].type() == typeid(int)) {
          type_size = sizeof(int);
        } else if (value_vec[0].type() == typeid(float)) {
          type_size = sizeof(float);
        } else if (value_vec[0].type() == typeid(char)) {
          type_size = sizeof(char);
        }
        auto byteLength = value_vec.size() * type_size;
        total_attr_vec_size += byteLength;
        // Everything goes into the same buffer
        bf_attr.buffer     = buffer_idx;
        bf_attr.byteOffset = byteOffset_feature_attributes;
        bf_attr.byteLength = byteLength;
        bf_attr.byteStride = 0; // all feature attribute value arrays need to be tightly packed https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#binary-table-format
        bf_attr.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
        id_bf_attr = model.bufferViews.size();
        model.bufferViews.push_back(bf_attr);
        byteOffset_feature_attributes += byteLength;
      }
      if (value_vec[0].type() == typeid(char)) {
        tinygltf::BufferView bf_attr;
        auto& value_vec_offs = feature_attribute_string_offsets[name];
        value_vec_offs.offsets.shrink_to_fit();
        auto byteLength = value_vec_offs.offsets.size() * sizeof(unsigned);
        total_attr_vec_size += byteLength;
        // Everything goes into the same buffer
        bf_attr.buffer     = buffer_idx;
        bf_attr.byteOffset = byteOffset_feature_attributes;
        bf_attr.byteLength = byteLength;
        bf_attr.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
        id_bf_attr_offset = model.bufferViews.size();
        model.bufferViews.push_back(bf_attr);
        byteOffset_feature_attributes += byteLength;
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
    property_table["class"] = tinygltf::Value((std::string)metadata_class_name_);
    property_table["count"] = tinygltf::Value((int)info_vec.size()); // feature count
    property_table["name"] = tinygltf::Value((std::string)"propertyTable name");
    property_table["properties"] = tinygltf::Value(properties_property_tbl);

    tinygltf::Value::Object ext_structural_metadata_object;
    ext_structural_metadata_object["schema"] = tinygltf::Value(metadata_schema);
    tinygltf::Value::Array property_tables;
    property_tables.emplace_back(property_table);
    ext_structural_metadata_object["propertyTables"] = tinygltf::Value(property_tables);
    tinygltf::ExtensionMap root_extensions;
    root_extensions["EXT_structural_metadata"] = tinygltf::Value(ext_structural_metadata_object);

    // Copy contents to the buffer
    buffer.data.resize(index_vec.size() * sizeof(unsigned) +
                       vertex_vec.size() * 6 * sizeof(float) +
                       feature_id_vec.size() * sizeof(float) +
                       total_attr_vec_size);
    auto ptr_end_of_data = buffer.data.data();
    memcpy(ptr_end_of_data,
           (unsigned char*)index_vec.data(),
           index_vec.size() * sizeof(unsigned));
    ptr_end_of_data += index_vec.size() * sizeof(unsigned);
    memcpy(ptr_end_of_data,
           (unsigned char*)vertex_vec[0].data(),
           vertex_vec.size() * 6 * sizeof(float));
    ptr_end_of_data += vertex_vec.size() * 6 * sizeof(float);
    memcpy(ptr_end_of_data,
           (unsigned char*)feature_id_vec.data(),
           feature_id_vec.size() * sizeof(float));
    ptr_end_of_data += feature_id_vec.size() * sizeof(float);
    for (const auto& [name, value_vec] : feature_attribute_map) {
      unsigned long type_size(0);
      if (value_vec[0].type() == typeid(bool)) {
        type_size = sizeof(bool);
      } else if (value_vec[0].type() == typeid(int)) {
        type_size = sizeof(int);
        std::vector<int> value_vec_int;
        value_vec_int.reserve(value_vec.size());
        for (auto& i : value_vec) {
          value_vec_int.emplace_back(std::any_cast<int>(i));
        }
        value_vec_int.shrink_to_fit();
        auto value_vec_int_size = value_vec_int.size() * type_size;
        memcpy(ptr_end_of_data,
               (unsigned char*)value_vec_int.data(),
               value_vec_int_size);
        ptr_end_of_data += value_vec_int_size;
      } else if (value_vec[0].type() == typeid(float)) {
        type_size = sizeof(float);
        std::vector<float> value_vec_float;
        value_vec_float.reserve(value_vec.size());
        for (auto& i : value_vec) {
          value_vec_float.emplace_back(std::any_cast<float>(i));
        }
        value_vec_float.shrink_to_fit();
        auto value_vec_float_size = value_vec_float.size() * type_size;
        memcpy(ptr_end_of_data,
               (unsigned char*)value_vec_float.data(),
               value_vec_float_size);
        ptr_end_of_data += value_vec_float_size;
      } else if (value_vec[0].type() == typeid(char)) {
        // Copy the attribute values
        type_size = sizeof(char);
        std::vector<char> value_vec_char;
        value_vec_char.reserve(value_vec.size());
        for (auto& i : value_vec) {
          value_vec_char.emplace_back(std::any_cast<char>(i));
        }
        value_vec_char.shrink_to_fit();
        auto value_vec_char_size = value_vec_char.size() * type_size;
        memcpy(ptr_end_of_data,
               (unsigned char*)value_vec_char.data(),
               value_vec_char_size);
        ptr_end_of_data += value_vec_char_size;
        // Copy the string offsets
        type_size = sizeof(unsigned);
        auto stringoffset = feature_attribute_string_offsets[name];
        auto offset_vec_size = stringoffset.offsets.size() * type_size;
        memcpy(ptr_end_of_data,
               (unsigned char*)stringoffset.offsets.data(),
               offset_vec_size);
        ptr_end_of_data += offset_vec_size;
      }
    }

    model.meshes.push_back(mesh);
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
        c[0],  c[2], -c[1],  1
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
    model.buffers.push_back(buffer);
    model.extensions = root_extensions;
    model.extensionsUsed.emplace_back("EXT_mesh_features");
    model.extensionsUsed.emplace_back("EXT_structural_metadata");

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