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

  tinygltf::Value create_ext_mesh_features(int featureCount,
                                                  int attribute,
                                                  int propertyTable)
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
    featureId_set[ "attribute"] = tinygltf::Value(attribute);
    // This feature ID set is associated with the propertyTable at the index.
    featureId_set[ "propertyTable"] = tinygltf::Value(propertyTable);
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
    typedef std::array<float,6> arr6f;
    struct TCInfo {
      size_t i_input;
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
    std::vector<float> feature_id_vec; // feature id vertex attribute
    std::vector<unsigned> index_vec;
    std::vector<TCInfo>   info_vec;
    // TODO: we need to protect against invalid values, eg. bouwjaar=-1 or so
    // TODO: these three vectors are hardcoded for now, for testing. Normally,
    //  we would want to construct these vectors dynamically, based on what
    //  attribute inputs we get.
    std::vector<unsigned> attr_1_vec; // objectid
    // We store the string attributes in a contiguous vector of char-s. This
    // makes it simpler to calculate its size and copy it to the buffer.
    std::vector<char>          attr_2_vec; // bagpandid
    unsigned long              string_offset            = 0;
    std::vector<unsigned long> attr_2_string_offset_vec = {
      string_offset,
    }; // offsets for strings
       // https://github.com/CesiumGS/3d-tiles/tree/main/specification/Metadata#strings
    std::vector<unsigned short> attr_3_vec; // bouwjaar
    unsigned                    v_offset = 0;
    unsigned                    i_offset = 0;
    // It's a float, because glTF accessors do not support UNSIGNED_INT types
    // for 32-bit integers in vertex attributes. There is UNSIGNED_SHORT, but
    // probably there could be more than ~65k individual features that we need
    // to identify in one gltf file.
    float feature_id = 0.0;
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

    for (unsigned i = 0; i < triangle_collections_inp.size(); ++i) {
      if (!triangle_collections_inp.get_data_vec()[i].has_value())
        continue;
      auto tc = triangle_collections_inp.get<TriangleCollection>(i);
      if (tc.vertex_count() == 0)
        continue;
      auto& normals = normals_inp.get<vec3f>(i);

      TCInfo inf;
      inf.i_input = i;
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
      feature_id += 1;
      // TODO: the feature ID should be selectable (eg it should be the
      // 'objectid' so that a feature can reference an external object)
      // add feature attribute values
      attr_1_vec.push_back(i);
      attr_3_vec.push_back(9999);
      // for string attributes, we also need to keep track of the offsets
      std::string string_attr = "0603100000023033";
      string_offset += string_attr.size() * sizeof(char);
      for (auto s : string_attr) {
        attr_2_vec.emplace_back(s);
      }
      attr_2_string_offset_vec.push_back(string_offset);

      info_vec.push_back(inf);
    }
    manager.clear_rev_crs_transform();

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

      bf_indices.buffer     = 0;
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
      // FIXME: "Declared minimum value for this component (-0.7280862927436829) does not match actual minimum (-0.6278660297393799)."
      acc_indices.minValues     = { inf.index_min };
      acc_indices.maxValues     = { inf.index_max };
      model.accessors.push_back(acc_indices);
      primitive.indices = model.accessors.size() - 1;

      bf_attributes.buffer = 0;
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
      acc_normals.minValues     = inf.normals_min;
      acc_normals.maxValues     = inf.normals_max;
      model.accessors.push_back(acc_normals);
      primitive.attributes["NORMAL"] =
        model.accessors.size() - 1; // The index of the accessor for normals

      bf_feature_ids.buffer = 0;
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
      acc_feature_ids.count            = inf.vertex_count;
      // This is an int, because tinygltf::Value only knows 'int' (no unsigned etc).
      int id_acc_feature_ids = model.accessors.size();
      model.accessors.push_back(acc_feature_ids);
      std::string fid = "_FEATURE_ID_" + std::to_string(id_acc_feature_ids);
      primitive.attributes[fid] =
        model.accessors.size() - 1; // The index of the accessor for normals

      tinygltf::ExtensionMap primitive_extensions;
      primitive_extensions["EXT_mesh_features"] = create_ext_mesh_features(1, id_acc_feature_ids, 0);

      primitive.material   = get_material_id(ftype);
      primitive.mode       = TINYGLTF_MODE_TRIANGLES;
      primitive.extensions = primitive_extensions;
      mesh.primitives.push_back(primitive);
    }

    // EXT_structural_metadata
    // TODO: for each feature attribute, instead of hardcoded
    // Schema definition
    tinygltf::Value::Object prop_objectid;
    // because tinygltf::Value("somestring") thinks that "somestring" is a bool
    prop_objectid["description"] = tinygltf::Value((std::string)"objectid property description");
    prop_objectid["type"] = tinygltf::Value((std::string)"SCALAR");
    prop_objectid["componentType"] = tinygltf::Value((std::string)"UINT32");
    prop_objectid["required"] = tinygltf::Value(true);
    tinygltf::Value::Object prop_bagpandid;
    prop_bagpandid["description"] = tinygltf::Value((std::string)"bagpandid property description");
    prop_bagpandid["type"] = tinygltf::Value((std::string)"STRING");
    tinygltf::Value::Object prop_bouwjaar;
    prop_bouwjaar["description"] = tinygltf::Value((std::string)"bouwjaar property description");
    prop_bouwjaar["type"] = tinygltf::Value((std::string)"SCALAR");
    prop_bouwjaar["componentType"] = tinygltf::Value((std::string)"UINT16");
    tinygltf::Value::Object building_class_properties;
    building_class_properties["objectid"] = tinygltf::Value(prop_objectid);
    building_class_properties["bagpandid"] = tinygltf::Value(prop_bagpandid);
    building_class_properties["bouwjaar"] = tinygltf::Value(prop_bouwjaar);
    tinygltf::Value::Object building_class;
    building_class["name"] = tinygltf::Value((std::string)"Building");
    building_class["description"] = tinygltf::Value((std::string)"Building class description");
    building_class["properties"] = tinygltf::Value(building_class_properties);
    tinygltf::Value::Object classes;
    classes["building"] = tinygltf::Value(building_class);
    tinygltf::Value::Object schema;
    schema["classes"] = tinygltf::Value(classes);

    // Add the property tables for the feature attributes. The property tables
    // reference bufferviews, which reference the complete array of the values
    // of one feature attribute (column-based layout).
    // https://github.com/CesiumGS/glTF/tree/3d-tiles-next/extensions/2.0/Vendor/EXT_structural_metadata#ext_structural_metadata
    // We need a bufferView for each feature attribute, plus the string offsets
    // in case of string attributes.
    tinygltf::BufferView bf_attr_1;
    tinygltf::BufferView bf_attr_2_offsets;
    tinygltf::BufferView bf_attr_2;
    tinygltf::BufferView bf_attr_3;
    // Everything goes into the same buffer
    auto byteOffset_attr_1 =
      byteOffset_feature_id + feature_id_vec.size() * sizeof(float);
    bf_attr_1.buffer     = 0;
    bf_attr_1.byteOffset = byteOffset_attr_1;
    bf_attr_1.byteLength = attr_1_vec.size() * sizeof(unsigned);
    bf_attr_1.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
    auto id_bf_attr_1    = model.bufferViews.size();
    model.bufferViews.push_back(bf_attr_1);

    auto byteOffset_attr_2_offsets =
      byteOffset_attr_1 + attr_1_vec.size() * sizeof(unsigned);
    bf_attr_2_offsets.buffer     = 0;
    bf_attr_2_offsets.byteOffset = byteOffset_attr_2_offsets;
    bf_attr_2_offsets.byteLength =
      attr_2_string_offset_vec.size() * sizeof(unsigned long);
    bf_attr_2_offsets.target  = TINYGLTF_TARGET_ARRAY_BUFFER;
    auto id_bf_attr_2_offsets = model.bufferViews.size();
    model.bufferViews.push_back(bf_attr_2_offsets);

    auto byteOffset_attr_2 =
      byteOffset_attr_2_offsets +
      attr_2_string_offset_vec.size() * sizeof(unsigned long);
    bf_attr_2.buffer     = 0;
    bf_attr_2.byteOffset = byteOffset_attr_2;
    bf_attr_2.byteLength = attr_2_vec.size() * sizeof(char);
    bf_attr_2.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
    auto id_bf_attr_2    = model.bufferViews.size();
    model.bufferViews.push_back(bf_attr_2);

    auto byteOffset_attr_3 =
      byteOffset_attr_2 + attr_3_vec.size() * sizeof(unsigned short);
    bf_attr_3.buffer     = 0;
    bf_attr_3.byteOffset = byteOffset_attr_3;
    bf_attr_3.byteLength = attr_3_vec.size() * sizeof(unsigned short);
    bf_attr_3.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
    auto id_bf_attr_3    = model.bufferViews.size();
    model.bufferViews.push_back(bf_attr_3);

    tinygltf::Value::Object prop_tbl_attr_1;
    prop_tbl_attr_1["values"] = tinygltf::Value((int)id_bf_attr_1);
    tinygltf::Value::Object prop_tbl_attr_2;
    prop_tbl_attr_2["values"] = tinygltf::Value((int)id_bf_attr_2);
    prop_tbl_attr_2["stringOffsets"] = tinygltf::Value((int)id_bf_attr_2_offsets);
    prop_tbl_attr_2["stringOffsetType"] = tinygltf::Value((std::string)"UINT64");
    tinygltf::Value::Object prop_tbl_attr_3;
    prop_tbl_attr_3["values"] = tinygltf::Value((int)id_bf_attr_3);
    tinygltf::Value::Object properties_building;
    properties_building["objectid"] = tinygltf::Value(prop_tbl_attr_1);
    properties_building["bagpandid"] = tinygltf::Value(prop_tbl_attr_2);
    properties_building["bouwjaar"] = tinygltf::Value(prop_tbl_attr_3);
    tinygltf::Value::Object property_table_building;
    property_table_building["class"] = tinygltf::Value((std::string)"building");
    property_table_building["count"] = tinygltf::Value((int)attr_1_vec.size());
    property_table_building["name"] = tinygltf::Value((std::string)"building propertyTable name");
    property_table_building["properties"] = tinygltf::Value(properties_building);

    tinygltf::Value::Object ext_structural_metadata_object;
    ext_structural_metadata_object["schema"] = tinygltf::Value(schema);
    tinygltf::Value::Array property_tables;
    property_tables.emplace_back(property_table_building);
    ext_structural_metadata_object["proptertyTables"] = tinygltf::Value(property_tables);
    tinygltf::ExtensionMap root_extensions;
    root_extensions["EXT_structural_metadata"] = tinygltf::Value(ext_structural_metadata_object);


    // Copy contents to the buffer
    // TODO: we won't actually know upfront how many attribute vectors we have and
    // what is their type
    unsigned long total_attr_vec_size =
      (attr_1_vec.size() * sizeof(unsigned) +
       attr_3_vec.size() * sizeof(unsigned short) +
       attr_2_string_offset_vec.size() * sizeof(unsigned long) +
       attr_2_vec.size() * sizeof(char));

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
    // TODO: For each attribute vector, we have to copy the contents to the buffer
    ptr_end_of_data += feature_id_vec.size() * sizeof(float);
    memcpy(ptr_end_of_data,
           (unsigned char*)attr_1_vec.data(),
           attr_1_vec.size() * sizeof(unsigned));
    ptr_end_of_data += attr_1_vec.size() * sizeof(unsigned);
    memcpy(ptr_end_of_data,
           (unsigned char*)attr_2_string_offset_vec.data(),
           attr_2_string_offset_vec.size() * sizeof(unsigned long));
    ptr_end_of_data += attr_2_string_offset_vec.size() * sizeof(unsigned long);
    memcpy(ptr_end_of_data,
           (unsigned char*)attr_2_vec.data(),
           attr_2_vec.size() * sizeof(char));
    ptr_end_of_data += attr_2_vec.size() * sizeof(char);
    memcpy(ptr_end_of_data,
           (unsigned char*)attr_3_vec.data(),
           attr_3_vec.size() * sizeof(unsigned short));

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
    if(relative_to_center) {
      node.matrix = {
        1,  0,  0,  0,
        0,  0, -1,  0,
        0,  1,  0,  0,
        c[0],  c[1],  c[2],  1
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