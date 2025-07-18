
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
#include <ctime>
#include <geoflow/common.hpp>
#include <geoflow/geoflow.hpp>
#include <regex>
#include <nlohmann/json.hpp>
#include <string>

namespace geoflow::nodes::basic3d
{
  static std::unordered_map <std::string, int> st_map =
  {
    {"GroundSurface", 0},
    {"RoofSurface",1},
    {"WallSurface", 2},
    {"ClosureSurface", 3},
    {"OuterCeilingSurface", 4},
    {"OuterFloorSurface", 5},
    {"Window", 6},
    {"Door", 7}
  };

  bool calculate_kwaliteitsindicator(const float& b3_bag_bag_overlap, const std::string& b3_val3dity_lod22, const std::string& b3_pw_selectie_reden) {
    return
        (b3_bag_bag_overlap == 0) &&
        (b3_val3dity_lod22 == "[]") &&
        (
            (b3_pw_selectie_reden != "_HIGHEST_YET_INSUFFICIENT_COVERAGE") &&
            (b3_pw_selectie_reden != "_LATEST_BUT_OUTDATED")
        );
  }

  void CityJSONReaderNode::process() {
    // get filepath from paramter

    // read json file from disk
    std::ifstream inputStream(filepath_);
    nlohmann::json json;
    try {
      inputStream >> json;
    } catch (const std::exception& e) {
      throw(gfIOError(e.what()));
    }

    // extract geometries
    // WARNING: this is code is only written to work with the dataset 'DenHaag_01.json', expect crashes with other files
    std::vector<std::vector<double>> verts = json["vertices"];
    std::vector<double> scale = json["transform"]["scale"];

    auto& faces = vector_output("faces");
    auto& surface_types = vector_output("surface_types");
    for (const auto& cobject : json["CityObjects"]) {
      // iterate all geometries
      for (const auto& geom : cobject["geometry"]) {
        if (
          geom["type"] == "Solid" && // only care about solids
          geom["lod"] == extract_lod_ // of this LoD
        ) {
          size_t face_cnt = 0;
          // get faces of exterior shell
          for (const auto& ext_face : geom["boundaries"][0]) {
            LinearRing ring;
            for (const auto& i : ext_face[0]) { // get vertices of outer rings
              ring.push_back({
                float(verts[i][0] * scale[0]),
                float(verts[i][1] * scale[1]),
                float(verts[i][2] * scale[2])
              });
              // get the surface type
            }
            int value = geom["semantics"]["values"][0][face_cnt++];
            const std::string type_string = geom["semantics"]["surfaces"][value]["type"];
            surface_types.push_back(st_map[type_string]);
            faces.push_back(ring);
          }
        }
      }
    }
  }

  template<typename T> void add_vertices_ring(std::map<arr3d, size_t>& vertex_map, std::vector<arr3d>& vertex_vec, std::set<arr3d>& vertex_set, const T& ring, NodeManager& manager, Box& bbox) {
    size_t v_cntr = vertex_vec.size();
    for (auto &vertex_ : ring)
    {
      auto vertex = manager.coord_transform_rev(vertex_);
      bbox.add(vertex);
      auto [it, did_insert] = vertex_set.insert(vertex);
      if (did_insert)
      {
        vertex_map[vertex] = v_cntr++;
        vertex_vec.push_back(vertex);
      }
    }
  }

  Box add_vertices_polygon(std::map<arr3d, size_t>& vertex_map, std::vector<arr3d>& vertex_vec, std::set<arr3d>& vertex_set, const LinearRing& polygon, NodeManager& manager) {
    Box bbox;
    add_vertices_ring(vertex_map, vertex_vec, vertex_set, polygon, manager, bbox);
    for (auto &iring : polygon.interior_rings()) {
      add_vertices_ring(vertex_map, vertex_vec, vertex_set, iring, manager, bbox);
    }
    return bbox;
  }

  Box add_vertices_mesh(std::map<arr3d, size_t>& vertex_map, std::vector<arr3d>& vertex_vec, std::set<arr3d>& vertex_set, const Mesh& mesh, NodeManager& manager) {
    Box bbox;
    for (auto &face : mesh.get_polygons())
    {
      bbox.add( add_vertices_polygon(vertex_map, vertex_vec, vertex_set, face, manager) );
    }
    return bbox;
  }


    // Helper functions for processing CityJSON data
  class CityJSON{

    public:
      static std::vector<std::vector<size_t>> LinearRing2jboundary(std::map<arr3d, size_t>& vertex_map, const LinearRing& face, NodeManager& manager);
      static nlohmann::json::object_t mesh2jSolid(const Mesh& mesh, const char* lod, std::map<arr3d, size_t>& vertex_map, NodeManager& manager);
      static void write_cityobjects(gfSingleFeatureInputTerminal& footprints,
                                    gfSingleFeatureInputTerminal& multisolids_lod12,
                                    gfSingleFeatureInputTerminal& multisolids_lod13,
                                    gfSingleFeatureInputTerminal& multisolids_lod22,
                                    gfMultiFeatureInputTerminal&  attributes,
                                    gfMultiFeatureInputTerminal&  part_attributes,
                                    json&                         outputJSON,
                                    std::vector<arr3d>&           vertex_vec,
                                    std::string&                  identifier_attribute,
                                    StrMap&                       output_attribute_names,
                                    bool&                         only_output_renamed,
                                    NodeManager&                  node_manager);
      static void write_to_file(const json& outputJSON, fs::path& fname, bool prettyPrint_);
      static nlohmann::json::array_t compute_geographical_extent(Box& bbox, NodeManager& manager);
  };

  std::vector<std::vector<size_t>> CityJSON::LinearRing2jboundary(std::map<arr3d, size_t>& vertex_map, const LinearRing& face, NodeManager& manager) {
    std::vector<std::vector<size_t>> jface;
    std::vector<size_t> exterior_ring;
    for (auto &vertex_ : face) {
      auto vertex = manager.coord_transform_rev(vertex_);
      exterior_ring.push_back(vertex_map[vertex]);
    }
    jface.emplace_back(std::move(exterior_ring));
    for (auto &iring : face.interior_rings()) {
      std::vector<size_t> interior_ring;
      for (auto &vertex_ : iring) {
        auto vertex = manager.coord_transform_rev(vertex_);
        interior_ring.push_back(vertex_map[vertex]);
      }
      jface.emplace_back(std::move(interior_ring));
    }
    return jface;
  }

  nlohmann::json::object_t CityJSON::mesh2jSolid(const Mesh& mesh, const char* lod, std::map<arr3d, size_t>& vertex_map, NodeManager& manager) {
    auto geometry = nlohmann::json::object();
    geometry["type"] = "Solid";
    geometry["lod"] = lod;
    std::vector<std::vector<std::vector<size_t>>> exterior_shell;

    for (auto &face : mesh.get_polygons())
    {
      exterior_shell.emplace_back( LinearRing2jboundary(vertex_map, face, manager) );
    }
    geometry["boundaries"] = {exterior_shell};

    auto surfaces = nlohmann::json::array();
    surfaces.push_back(nlohmann::json::object({{
      "type", "GroundSurface"
    }}));
    surfaces.push_back(nlohmann::json::object({{
      "type", "RoofSurface"
    }}));
    surfaces.push_back(nlohmann::json::object({
      {
        "type", "WallSurface"
      },
      {
        "on_footprint_edge", true
      }
    }));
    surfaces.push_back(nlohmann::json::object({
      {
        "type", "WallSurface"
      },
      {
        "on_footprint_edge", false
      }
    }));
    geometry["semantics"] = {
      {"surfaces", surfaces},
      {"values", {mesh.get_labels()}}
    };
    return geometry;
  }

  void CityJSON::write_to_file(const json& outputJSON, fs::path& fname, bool prettyPrint_)
  {
    fs::create_directories(fname.parent_path());
    std::ofstream ofs;
    ofs.open(fname);
    // Note that the following line has no effect on the output
    ofs << std::fixed << std::setprecision(4);
    try {
      if (prettyPrint_)
        ofs << outputJSON.dump(2);
      else
        ofs << outputJSON;
    } catch (const std::exception& e) {
      throw(gfIOError(e.what()));
    }
  }

  // Computes the geographicalExtent array from a geoflow::Box and the data_offset from the NodeManager
  nlohmann::json::array_t CityJSON::compute_geographical_extent(Box& bbox, NodeManager& manager) {
    auto minp = bbox.min();
    auto maxp = bbox.max();
    return {
      minp[0],
      minp[1],
      minp[2],
      maxp[0],
      maxp[1],
      maxp[2]
    };
  }

  void CityJSON::write_cityobjects(
    gfSingleFeatureInputTerminal& footprints,
    gfSingleFeatureInputTerminal& multisolids_lod12,
    gfSingleFeatureInputTerminal& multisolids_lod13,
    gfSingleFeatureInputTerminal& multisolids_lod22,
    gfMultiFeatureInputTerminal&  attributes,
    gfMultiFeatureInputTerminal&  part_attributes,
    json&                         outputJSON,
    std::vector<arr3d>&           vertex_vec,
    std::string&                  identifier_attribute,
    StrMap&                       output_attribute_names,
    bool&                         only_output_renamed,
    NodeManager&                  node_manager)
  {
    std::map<arr3d, size_t> vertex_map;
    std::set<arr3d> vertex_set;
    size_t id_cntr = 0;
    size_t bp_counter = 0;

    // we expect at least one of the geomtry inputs is set
    bool export_lod12 = multisolids_lod12.has_data();
    bool export_lod13 = multisolids_lod13.has_data();
    bool export_lod22 = multisolids_lod22.has_data();
    size_t geometry_count = 0;
    if (export_lod12)
      geometry_count = multisolids_lod12.size();
    else if (export_lod13)
      geometry_count = multisolids_lod13.size();
    else if (export_lod22)
      geometry_count = multisolids_lod22.size();

    typedef std::unordered_map<int, Mesh> MeshMap;

    for (size_t i=0; i<geometry_count; ++i) {
      auto building = nlohmann::json::object();
      auto b_id = std::to_string(++id_cntr);
      building["type"] = "Building";

      // Building atributes
      bool id_from_attr = false;
      auto jattributes = nlohmann::json::object();
      for (auto& term : attributes.sub_terminals()) {
        auto tname = term->get_full_name();

        //see if we need to rename this attribute
        auto search = output_attribute_names.find(tname);
        if(search != output_attribute_names.end()) {
          //ignore if the new name is an empty string
          if(search->second.size()!=0)
            tname = search->second;
        } else if (only_output_renamed) {
          continue;
        }

        if (!term->get_data_vec()[i].has_value()) {
          nlohmann::json j_null;
          jattributes[tname] = j_null;
          continue;
        }

        if (term->accepts_type(typeid(bool))) {
          jattributes[tname] = term->get<const bool&>(i);
        } else if (term->accepts_type(typeid(float))) {
          jattributes[tname] = term->get<const float&>(i);
          if (tname == identifier_attribute) {
            b_id = std::to_string(term->get<const float&>(i));
          }
        } else if (term->accepts_type(typeid(int))) {
          jattributes[tname] = term->get<const int&>(i);
          if (tname == identifier_attribute) {
            b_id = std::to_string(term->get<const int&>(i));
            id_from_attr = true;
          }
        } else if (term->accepts_type(typeid(std::string))) {
          jattributes[tname] = term->get<const std::string&>(i);
          if (tname == identifier_attribute) {
            b_id = term->get<const std::string&>(i);
            id_from_attr = true;
          }
        // for date/time we follow https://en.wikipedia.org/wiki/ISO_8601
        } else if (term->accepts_type(typeid(Date))) {
          auto t = term->get<const Date&>(i);
          std::string date = t.format_to_ietf();
          jattributes[tname] = date;
        } else if (term->accepts_type(typeid(Time))) {
          auto t = term->get<const Time&>(i);
          std::string time = std::to_string(t.hour) + ":" + std::to_string(t.minute) + ":" + std::to_string(t.second) + "Z";
          jattributes[tname] = time;
        } else if (term->accepts_type(typeid(DateTime))) {
          auto t = term->get<const DateTime&>(i);
          std::string datetime = t.format_to_ietf();
          jattributes[tname] = datetime;
        }
      }

      building["attributes"] = jattributes;

      // footprint geometry
      auto fp_geometry = nlohmann::json::object();
      fp_geometry["lod"] = "0";
      fp_geometry["type"] = "MultiSurface";

      LinearRing footprint = footprints.get<LinearRing>(i);
      add_vertices_polygon(vertex_map, vertex_vec, vertex_set, footprint, node_manager);
      fp_geometry["boundaries"] = {CityJSON::LinearRing2jboundary(vertex_map, footprint, node_manager)};
      building["geometry"].push_back(fp_geometry);

      std::vector<std::string> buildingPartIds;


      bool has_solids = false;
      if (export_lod12) has_solids = multisolids_lod12.get_data_vec()[i].has_value();
      if (export_lod13) has_solids = multisolids_lod13.get_data_vec()[i].has_value();
      if (export_lod22) has_solids = multisolids_lod22.get_data_vec()[i].has_value();

      Box building_bbox;
      if (has_solids) {
        MeshMap meshmap;
        if (export_lod22) {
          meshmap = multisolids_lod22.get<MeshMap>(i);
        } else if (export_lod13) {
          meshmap = multisolids_lod13.get<MeshMap>(i);
        } else if (export_lod12) {
          meshmap = multisolids_lod12.get<MeshMap>(i);
        }
        for ( const auto& [sid, solid_lodx] : meshmap ) {
          auto buildingPart = nlohmann::json::object();
          auto bp_id = b_id + "-" + std::to_string(sid);

          buildingPartIds.push_back(bp_id);
          buildingPart["type"] = "BuildingPart";
          buildingPart["parents"] = {b_id};

          // Use try-except here for some rare cases when the sid's between different lod's do not line up (eg for very fragmented buildings from poor dim pointcloud).
          if (export_lod12) {
            try {
              building_bbox = add_vertices_mesh(vertex_map, vertex_vec, vertex_set, multisolids_lod12.get<MeshMap>(i).at(sid), node_manager);
              buildingPart["geometry"].push_back(CityJSON::mesh2jSolid(multisolids_lod12.get<MeshMap>(i).at(sid), "1.2", vertex_map, node_manager));
            } catch (const std::exception& e) {
              std::cout << "skipping lod 12 building part\n";
            }
          }
          if (export_lod13) {
            try {
              building_bbox = add_vertices_mesh(vertex_map, vertex_vec, vertex_set, multisolids_lod13.get<MeshMap>(i).at(sid), node_manager);
              buildingPart["geometry"].push_back(CityJSON::mesh2jSolid(multisolids_lod13.get<MeshMap>(i).at(sid), "1.3", vertex_map, node_manager));
            } catch (const std::exception& e) {
              std::cout << "skipping lod 13 building part\n";
            }
          }
          if (export_lod22) {
            building_bbox = add_vertices_mesh(vertex_map, vertex_vec, vertex_set, multisolids_lod22.get<MeshMap>(i).at(sid), node_manager);
            buildingPart["geometry"].push_back(CityJSON::mesh2jSolid(multisolids_lod22.get<MeshMap>(i).at(sid), "2.2", vertex_map, node_manager));
          }

          //attrubutes
          auto jattributes = nlohmann::json::object();
          for (auto& term : part_attributes.sub_terminals()) {
            auto tname = term->get_full_name();
            if (!term->get_data_vec()[i].has_value()) {
              nlohmann::json j_null;
              jattributes[tname] = j_null;
              continue;
            }

            if (term->accepts_type(typeid(bool))) {
              jattributes[tname] = term->get<const bool&>(bp_counter);
            } else if (term->accepts_type(typeid(float))) {
              jattributes[tname] = term->get<const float&>(bp_counter);
            } else if (term->accepts_type(typeid(int))) {
              jattributes[tname] = term->get<const int&>(bp_counter);
            } else if (term->accepts_type(typeid(std::string))) {
              jattributes[tname] = term->get<const std::string&>(bp_counter);
            }
          }
          ++bp_counter;
          buildingPart["attributes"] = jattributes;

          outputJSON["CityObjects"][bp_id] = buildingPart;
        }
      }

      building["children"] = buildingPartIds;
      building["geographicalExtent"] = CityJSON::compute_geographical_extent(building_bbox, node_manager);

      outputJSON["CityObjects"][b_id] = building;
    }
  }

  void CityJSONWriterNode::process() {
    // inputs
    auto& footprints = vector_input("footprints");
    auto& multisolids_lod12 = vector_input("geometry_lod12");
    auto& multisolids_lod13 = vector_input("geometry_lod13");
    auto& multisolids_lod22 = vector_input("geometry_lod22");
    auto& attributes = poly_input("attributes");
    auto& part_attributes = poly_input("part_attributes");
    std::string identifier_attribute =
      manager.substitute_globals(identifier_attribute_);

    auto CRS = manager.substitute_globals(CRS_).c_str();
    manager.set_rev_crs_transform(CRS);

    nlohmann::json outputJSON;

    outputJSON["type"] = "CityJSON";
    outputJSON["version"] = "2.0";
    outputJSON["CityObjects"] = nlohmann::json::object();

    std::vector<arr3d> vertex_vec;
    CityJSON::write_cityobjects(footprints,
                                multisolids_lod12,
                                multisolids_lod13,
                                multisolids_lod22,
                                attributes,
                                part_attributes,
                                outputJSON,
                                vertex_vec,
                                identifier_attribute,
                                output_attribute_names,
                                only_output_renamed_,
                                manager);

    Box bbox;

    std::vector<std::array<int,3>>vertices_int;
    for (auto& vertex : vertex_vec) {
      bbox.add(vertex);
    }

    auto center = bbox.center();
    for (auto& vertex : vertex_vec) {
      vertices_int.push_back({
        int( (vertex[0] - center[0]) * 1000 ),
        int( (vertex[1] - center[1]) * 1000 ),
        int( (vertex[2] - center[2]) * 1000 )
      });
    }
    outputJSON["vertices"] = vertices_int;
    outputJSON["transform"] = {
      {"translate", center},
      {"scale", {0.001, 0.001, 0.001}}
    };

    // metadata
    auto metadata = nlohmann::json::object();
    metadata["geographicalExtent"] = CityJSON::compute_geographical_extent(bbox, manager);

    metadata["identifier"] = manager.substitute_globals(meta_identifier_);

    // metadata.datasetPointOfContact - only add it if at least one of the parameters is filled
    auto contact = nlohmann::json::object();

    if (std::string val = manager.substitute_globals(meta_poc_contactName_); !val.empty()) { contact["contactName"] = val; }
    if (std::string val = manager.substitute_globals(meta_poc_email_); !val.empty()) { contact["emailAddress"] = val; }
    if (std::string val = manager.substitute_globals(meta_poc_phone_); !val.empty()) { contact["phone"] = val; }
    // if (std::string val = manager.substitute_globals(meta_poc_address_); !val.empty()) { contact["address"] = val; }
    if (std::string val = manager.substitute_globals(meta_poc_type_); !val.empty()) { contact["contactType"] = val; }
    if (std::string val = manager.substitute_globals(meta_poc_website_); !val.empty()) { contact["website"] = val; }

    metadata["pointOfContact"] = contact;

    if (std::string val = manager.substitute_globals(meta_referenceDate_); !val.empty()) {
      // find current date if none provided
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%d");
      meta_referenceDate_              = oss.str();
    }
    metadata["referenceDate"] = manager.substitute_globals(meta_referenceDate_);

    metadata["referenceSystem"] = manager.substitute_globals(meta_referenceSystem_);
    if(manager.has_process_crs()) {
      metadata["referenceSystem"] = "https://www.opengis.net/def/crs/" +manager.get_rev_crs_id_auth_name()+ "/0/" +manager.get_rev_crs_id_code();
    }

    metadata["title"] = manager.substitute_globals(meta_title_);

    outputJSON["metadata"] = metadata;

    fs::path fname = fs::path(manager.substitute_globals(filepath_));
    CityJSON::write_to_file(outputJSON, fname, prettyPrint_);
    manager.clear_rev_crs_transform();
  }

  void CityJSONFeatureWriterNode::process() {
    // inputs
    auto& footprints = vector_input("footprints");
    auto& multisolids_lod12 = vector_input("geometry_lod12");
    auto& multisolids_lod13 = vector_input("geometry_lod13");
    auto& multisolids_lod22 = vector_input("geometry_lod22");
    auto& attributes = poly_input("attributes");
    auto& part_attributes = poly_input("part_attributes");
    std::string identifier_attribute =
      manager.substitute_globals(identifier_attribute_);

    manager.set_rev_crs_transform(manager.substitute_globals(CRS_).c_str());

    nlohmann::json outputJSON;

    outputJSON["type"] = "CityJSONFeature";
    outputJSON["CityObjects"] = nlohmann::json::object();

    std::vector<arr3d> vertex_vec;
    CityJSON::write_cityobjects(footprints,
                                multisolids_lod12,
                                multisolids_lod13,
                                multisolids_lod22,
                                attributes,
                                part_attributes,
                                outputJSON,
                                vertex_vec,
                                identifier_attribute,
                                output_attribute_names,
                                only_output_renamed_,
                                manager);

    // The main Building is the parent object.
    // Bit of a hack. Ideally we would know exactly which ID we set,
    // instead of just iterating. But it is assumed that in case of writing to
    // CityJSONFeature there is only one parent CityObject.
    for (auto& el : outputJSON["CityObjects"].items()) {
      if (!el.value().contains(std::string("parents"))) {
        outputJSON["id"] = el.key();
      }
    };

    std::vector<std::array<int,3>>vertices_int;
    double _offset_x = translate_x_;
    double _offset_y = translate_y_;
    double _offset_z = translate_z_;
    for (auto& vertex : vertex_vec) {
      vertices_int.push_back({
        int( (vertex[0] - _offset_x ) / scale_x_ ),
        int( (vertex[1] - _offset_y ) / scale_y_ ),
        int( (vertex[2] - _offset_z ) / scale_z_ )
      });
    }
    outputJSON["vertices"] = vertices_int;

    fs::path fname = fs::path(manager.substitute_globals(filepath_));
    CityJSON::write_to_file(outputJSON, fname, prettyPrint_);
    manager.clear_rev_crs_transform();
  }


  void CityJSONFeatureMetadataWriterNode::process() {

    manager.set_rev_crs_transform(manager.substitute_globals(CRS_).c_str());

    nlohmann::json outputJSON;

    outputJSON["type"] = "CityJSON";
    outputJSON["version"] = "2.0";
    outputJSON["CityObjects"] = nlohmann::json::object();
    outputJSON["vertices"] = nlohmann::json::array();

    outputJSON["transform"] = {
      {"translate", { translate_x_, translate_y_, translate_z_ }},
      {"scale", { scale_x_, scale_y_, scale_z_}}
    };
    outputJSON["metadata"] = {
      {"referenceSystem", "https://www.opengis.net/def/crs/" +manager.get_rev_crs_id_auth_name()+ "/0/" +manager.get_rev_crs_id_code() }
    };

    fs::path fname = fs::path(manager.substitute_globals(filepath_));
    CityJSON::write_to_file(outputJSON, fname, prettyPrint_);
    manager.clear_rev_crs_transform();
  }

  void JSONReaderNode::process() {
    // read json file from disk
    std::ifstream inputStream(manager.substitute_globals(filepath_));
    nlohmann::json json;
    try {
      inputStream >> json;
      inputStream.close();
    } catch (const std::exception& e) {
      throw(gfIOError(e.what()));
    }
    vector_output("json").push_back(json);
  }

  void offset_indices(nlohmann::json& j, const size_t& offset){
    if (j.type() == nlohmann::json::value_t::number_unsigned) {
      j = j.get<size_t>() + offset;
    } else {
      for (auto& k : j) {
        offset_indices(k, offset);
      }
    }
  }
  void set_vertex_index_offset(nlohmann::json& geometry, const size_t& offset) {
    // std::cout<<offset<< std::endl;

    // std::cout<<geometry<< std::endl;
    if(offset==0) return;
    offset_indices(geometry["boundaries"], offset);
    // std::cout<<geometry<< std::endl;
  }

  void CityJSONLinesWriterNode::process() {
    auto jsonstr = input("first_line").get<std::string>();
    nlohmann::json metajson;
    try {
      metajson = nlohmann::json::parse(jsonstr);
    } catch (const std::exception& e) {
      throw(gfIOError(e.what()));
    }
    auto& features_inp = vector_input("features");

    size_t vindex_offset = 0;
    for (size_t i=0; i< features_inp.size(); ++i) {
      // std::cout<< "FI:" << i<< std::endl;
      auto& featurestr = features_inp.get<std::string>(i);
      if (featurestr.size()==0) {
        std::cout << "empty feature string for feature; skipping...\n";
        continue;
      }
      // std::cout<< featurestr << std::endl;
      nlohmann::json feature;
      try {
        feature = nlohmann::json::parse(featurestr);
      } catch (const std::exception& e) {
        throw(gfIOError(e.what()));
      }

      if(feature["type"] != "CityJSONFeature") {
        throw(gfException("input is not CityJSONFeature"));
      }

      for( auto [id, cobject] : feature["CityObjects"].items() ) {
        // std::cout<< "CID:" << id << std::endl;
        // std::cout<< "vertex_count:" << cobject[]<< std::endl;

        if (cobject["type"] == "Building") {
            if (
               cobject["attributes"].contains("b3_bag_bag_overlap") &&
               cobject["attributes"].contains("b3_val3dity_lod22") &&
               cobject["attributes"].contains("b3_pw_selectie_reden")
               ) {
                float b3_bag_bag_overlap = 0;
                if (cobject["attributes"]["b3_bag_bag_overlap"].is_number()) {
                    b3_bag_bag_overlap = cobject["attributes"].value("b3_bag_bag_overlap", 0);
                }
                std::string b3_val3dity_lod22 = cobject["attributes"].value("b3_val3dity_lod22", "[]");
                std::string b3_pw_selectie_reden = cobject["attributes"].value("b3_pw_selectie_reden", "");
                bool val = calculate_kwaliteitsindicator(b3_bag_bag_overlap, b3_val3dity_lod22, b3_pw_selectie_reden);
                cobject["attributes"]["b3_kwaliteitsindicator"] = val;
               }
        }

        metajson["CityObjects"][id] = cobject;
        //fix vertex indices...
        for (auto& geom : metajson["CityObjects"][id]["geometry"]) {
          set_vertex_index_offset(geom, vindex_offset);
          // std::cout<<boundaries<< std::endl;
        }
      }
      if(optimal_lod_) {
        for( auto& item : feature["CityObjects"].items() ) {
          auto& cobject = metajson["CityObjects"][item.key()];
          if(cobject["type"] == "BuildingPart") {
            auto& ref = cobject["parents"][0];
            std::string optilod = feature["CityObjects"][ ref ] ["attributes"]["optimal_lod"];
            std::vector<nlohmann::json> new_geometries;
            for(auto& geom : cobject["geometry"]) {
              if (geom["lod"] == optilod) {
                new_geometries.push_back(geom);
              }
            }
            cobject["geometry"] = new_geometries;
          }
        }
      }
      // std::cout << json << std::endl;
      metajson["vertices"].insert(metajson["vertices"].end(), feature["vertices"].begin(), feature["vertices"].end());
      // std::cout << json["vertices"] << std::endl;
      vindex_offset = metajson["vertices"].size();
    }

    // metadata
    auto& s = metajson["transform"]["scale"];
    auto& t = metajson["transform"]["translate"];

    if(recompute_offset_) {
      Box bbox;
      for(auto& v : metajson["vertices"]) {
        bbox.add(arr3f{
          (float) v[0].get<int>() * s[0].get<float>(),
          (float) v[1].get<int>() * s[1].get<float>(),
          (float) v[2].get<int>() * s[2].get<float>()
        });
      }
      auto c = bbox.center();
      t[0] = t[0].get<float>() + c[0];
      t[1] = t[1].get<float>() + c[1];
      t[2] = t[2].get<float>() + c[2];
      for(auto& v : metajson["vertices"]) {
        v[0] = int( v[0].get<double>() - double(c[0]/s[0].get<double>()) );
        v[1] = int( v[1].get<double>() - double(c[1]/s[1].get<double>()) );
        v[2] = int( v[2].get<double>() - double(c[2]/s[2].get<double>()) );
      }
    }

    Box bbox;
    //compute extent
    for(auto& v : metajson["vertices"]) {
      bbox.add(arr3f{
        (float) v[0].get<int>() * s[0].get<float>() + t[0].get<float>(),
        (float) v[1].get<int>() * s[1].get<float>() + t[1].get<float>(),
        (float) v[2].get<int>() * s[2].get<float>() + t[2].get<float>()
      });
    }
    metajson["metadata"]["geographicalExtent"] = CityJSON::compute_geographical_extent(bbox, manager);

    fs::path fname = fs::path(manager.substitute_globals(filepath_));
    CityJSON::write_to_file(metajson, fname, prettyPrint_);
  }

  std::set<std::string> split_string(const std::string& s, std::string delimiter) {
    std::set<std::string> parts;
    if (s.empty()) return parts;
    size_t last = 0;
    size_t next = 0;

    while ((next = s.find(delimiter, last)) != std::string::npos) {
      parts.insert(s.substr(last, next-last));
      last = next + 1;
    }
    parts.insert(s.substr(last));
    return parts;
  }

  std::vector<std::string> split_string_to_vec(const std::string& s, std::string delimiter) {
    std::vector<std::string> parts;
    if (s.empty()) return parts;
    size_t last = 0;
    size_t next = 0;

    while ((next = s.find(delimiter, last)) != std::string::npos) {
      parts.push_back(s.substr(last, next-last));
      last = next + 1;
    }
    parts.push_back(s.substr(last));
    return parts;
  }

  LinearRing CityJSONSurface2LinearRing(
    std::vector<std::vector<int>> face,
    const std::vector<std::vector<double>>& vertices,
    const std::vector<double>& jtranslate,
    const std::vector<double>& jscale,
    geoflow::NodeManager&  manager
  ) {
    LinearRing ring;
    for (const auto& i : face[0]) { // get vertices of exterior ring (skipping holes)
      ring.push_back(
        manager.coord_transform_fwd(
          double((vertices[i][0] * jscale[0])+jtranslate[0]),
          double((vertices[i][1] * jscale[1])+jtranslate[1]),
          double((vertices[i][2] * jscale[2])+jtranslate[2])
        )
      );
      // get the surface type
    }
    return ring;
  }

  void CityJSONL2MeshNode::process() {
    auto& meshes = vector_output("meshes");
    auto& roofparts = vector_output("roofparts");
    auto& attributes = poly_output("attributes");

    auto jsonstr = input("jsonl_metadata_str").get<std::string>();
    nlohmann::json metajson;
    try {
      metajson = nlohmann::json::parse(jsonstr);
    } catch (const std::exception& e) {
      throw(gfIOError(e.what()));
    }
    auto& features_inp = vector_input("jsonl_features_str");

    std::vector<double> jtranslate = metajson["transform"]["translate"];
    std::vector<double> jscale = metajson["transform"]["scale"];
    std::string referenceSystem = metajson["metadata"]["referenceSystem"];
    std::string epsg_code;

    std::regex re("\\d+$");
    std::smatch m;
    std::cout<< referenceSystem << std::endl;
    if(std::regex_search(referenceSystem, m, re)) {
      epsg_code = "EPSG:" + m[0].str();
      std::cout<< epsg_code << std::endl;
      manager.set_fwd_crs_transform(epsg_code.c_str());
    } else {
      throw(gfException("CRS not detected"));
    }

    auto feature_filter = split_string(manager.substitute_globals(cotypes), ",");
    for(auto& t : feature_filter) {
      std::cout << "filtering: " << t << std::endl;
    }
    bool filter_by_ftype = feature_filter.size() != 0;

    std::unordered_map<std::string, std::string> attribute_filter_map;
    auto attribute_filter = split_string(manager.substitute_globals(atribute_spec), ",");
    for(const auto& t : attribute_filter) {
      auto nt = split_string_to_vec(t, ":");
      if (nt.size()!=2) throw(gfException("Error in parsing attribute_spec string: "+manager.substitute_globals(atribute_spec)));
      attribute_filter_map[nt[0]] = nt[1];
    }

    if (bag3d_buildings_mode_) {
      auto& lod0_2d = vector_output("lod0_2d");
      auto& roofparts_lr = vector_output("roofparts_lr");

      auto& roofparts_lr_attributes = poly_output("roofparts_lr_attributes");
      roofparts_lr_attributes.add_vector("identificatie", typeid(std::string));
      roofparts_lr_attributes.add_vector("pand_deel_id", typeid(int));
      roofparts_lr_attributes.add_vector("dak_deel_id", typeid(int));

      auto& meshes_attributes = poly_output("meshes_attributes");
      meshes_attributes.add_vector("identificatie", typeid(std::string));
      meshes_attributes.add_vector("pand_deel_id", typeid(int));

      // add reliability indicator
      attributes.add_vector("b3_kwaliteitsindicator", typeid(bool));

      for (size_t i=0; i< features_inp.size(); ++i) {
        // std::cout<< "FI:" << i<< std::endl;
        auto& featurestr = features_inp.get<std::string>(i);
        if (featurestr.size()==0) {
          std::cout << "empty feature string for feature; skipping...\n";
          continue;
        }
        // std::cout<< featurestr << std::endl;
        nlohmann::json feature;
        try {
          feature = nlohmann::json::parse(featurestr);
        } catch (const std::exception& e) {
          throw(gfIOError(e.what()));
        }

        if(feature["type"] != "CityJSONFeature") {
          throw(gfException("input is not CityJSONFeature"));
        }
        std::vector<std::vector<double>> vertices = feature["vertices"];
        size_t n_attr=0, n_mesh=0;
        std::string optimal_lod_value = optimal_lod_value_;
        for( auto [id, cobject] : feature["CityObjects"].items() ) {
          // std::cout<< "CID:" << id << std::endl;
          // std::cout<< "vertex_count:" << cobject[]<< std::endl;

          if (cobject["type"] == "Building") {
            if (optimal_lod_) optimal_lod_value = cobject["attributes"]["optimal_lod"];

            if (cobject["attributes"].contains("b3_succes")) {
              if (cobject["attributes"]["b3_succes"].get<bool>() == false) {
                return;
              }
            }

            size_t n_children = 1;
            if (bag3d_attr_per_part_) n_children = cobject["children"].size();
            if (n_children==0) return;
            n_attr += n_children;
            // get_attributes
            for(auto& [jname, jval] : cobject["attributes"].items()) {
              if (jname == "b3_bag_bag_overlap") {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(float));
                }
                for (size_t i=0; i<n_children; ++i)
                  if (jval.is_null())
                    attributes.sub_terminal(jname).push_back(float(0));
                  else
                    attributes.sub_terminal(jname).push_back(jval.get<float>());
              } else if ( jname == "b3_bouwlagen" ||
                          jname == "b3_h_dak_min" ||
                          jname == "b3_h_dak_max" ||
                          jname == "b3_h_dak_50p" ||
                          jname == "b3_rmse_lod12" ||
                          jname == "b3_rmse_lod13" ||
                          jname == "b3_rmse_lod22" ||
                          jname == "b3_volume_lod12" ||
                          jname == "b3_volume_lod13" ||
                          jname == "b3_volume_lod22"
                          ) {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(float));
                }
                for (size_t i=0; i<n_children; ++i)
                  if (jval.is_null())
                    attributes.sub_terminal(jname).push_back_any(std::any());
                  else
                    attributes.sub_terminal(jname).push_back(jval.get<float>());
              } else if (jname == "b3_n_vlakken") {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(int));
                }
                for (size_t i=0; i<n_children; ++i)
                  if (jval.is_null())
                    attributes.sub_terminal(jname).push_back_any(std::any());
                  else
                    attributes.sub_terminal(jname).push_back(jval.get<int>());
              } else if(jval.is_string()) {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(std::string));
                }
                for (size_t i=0; i<n_children; ++i)
                  attributes.sub_terminal(jname).push_back(jval.get<std::string>());
              } else if (jval.is_number()) {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(float));
                }
                for (size_t i=0; i<n_children; ++i)
                  attributes.sub_terminal(jname).push_back(jval.get<float>());
              } else if (jval.is_boolean()) {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(bool));
                }
                for (size_t i=0; i<n_children; ++i)
                  attributes.sub_terminal(jname).push_back(jval.get<bool>());
              } else {
                if (!attributes.has_sub_terminal(jname)) {
                  attributes.add_vector(jname, typeid(std::string));
                }
                for (size_t i=0; i<n_children; ++i)
                  if(jval.is_null()) {
                    attributes.sub_terminal(jname).push_back_any(std::any());
                  } else {
                    attributes.sub_terminal(jname).push_back(jval.get<std::string>());
                  }
              }
            }
            // kwaliteitsindicator
            // std::cout << "b3_bag_bag_overlap: " << cobject["attributes"].contains("b3_bag_bag_overlap") << std::endl;
            // std::cout << "b3_val3dity_lod22: " << cobject["attributes"].contains("b3_val3dity_lod22") << std::endl;
            // std::cout << "b3_pw_selectie_reden: " << cobject["attributes"].contains("b3_pw_selectie_reden") << std::endl;
            if (
               cobject["attributes"].contains("b3_bag_bag_overlap") &&
               cobject["attributes"].contains("b3_val3dity_lod22") &&
               cobject["attributes"].contains("b3_pw_selectie_reden")
               ) {
                float b3_bag_bag_overlap = attributes.sub_terminal("b3_bag_bag_overlap").get<float>(attributes.sub_terminal("b3_bag_bag_overlap").size()-1);
                std::string b3_val3dity_lod22 = cobject["attributes"].value("b3_val3dity_lod22", "[]");
                std::string b3_pw_selectie_reden = cobject["attributes"].value("b3_pw_selectie_reden", "");
                bool val = calculate_kwaliteitsindicator(b3_bag_bag_overlap, b3_val3dity_lod22, b3_pw_selectie_reden);
                for (size_t i=0; i<n_children; ++i) attributes.sub_terminal("b3_kwaliteitsindicator").push_back(val);
            } else {
              for (size_t i=0; i<n_children; ++i) attributes.sub_terminal("b3_kwaliteitsindicator").push_back_any(std::any());
            }

            // get lod0 polygon
            auto& geom = cobject["geometry"][0];
            LinearRing ring;
            if(geom["type"] == "MultiSurface" && geom["lod"] == "0") {
              ring = CityJSONSurface2LinearRing( geom["boundaries"][0], vertices, jtranslate, jscale, manager );
              lod0_2d.push_back(ring);
            } else {
              throw(gfException("Building geometry has unexpected form"));
            }
          }
        }

        for( auto [id, cobject] : feature["CityObjects"].items() ) {
          // std::cout<< "CID:" << id << std::endl;
          // std::cout<< "vertex_count:" << cobject[]<< std::endl;

          if (cobject["type"] == "BuildingPart") {
            auto id_vec = split_string_to_vec(id, "-");
            std::string identificatie = id_vec.front();
            int part_id = std::stoi(id_vec.back());
            bool optimal_lod_geometry_found = false;
            for (const auto& geom : cobject["geometry"]) {

              if(geom["lod"].get<std::string>() == optimal_lod_value) {
                // get mesh
                if (
                  geom["type"] == "Solid"// only care about solids
                ) {
                  optimal_lod_geometry_found = true;
                  Mesh mesh;
                  Mesh roofparts_;
                  // get faces of exterior shell
                  unsigned face_i=0;
                  int roofpart_i=0;
                  for (const auto& ext_face : geom["boundaries"][0]) {
                    LinearRing ring;
                    for (const auto& i : ext_face[0]) { // get vertices of outer rings
                      ring.push_back(
                        manager.coord_transform_fwd(
                          double((vertices[i][0] * jscale[0])+jtranslate[0]),
                          double((vertices[i][1] * jscale[1])+jtranslate[1]),
                          double((vertices[i][2] * jscale[2])+jtranslate[2])
                        )
                      );
                      // get the surface type
                    }
                    int sindex = geom["semantics"]["values"][0][face_i++].get<int>();
                    auto& semobject = geom["semantics"]["surfaces"][ sindex ];
                    if (semobject["type"].get<std::string>() == "RoofSurface") {
                      roofparts_.push_polygon(ring, 2);
                      roofparts_lr.push_back(ring);
                      roofparts_lr_attributes.sub_terminal("identificatie").push_back(identificatie);
                      roofparts_lr_attributes.sub_terminal("pand_deel_id").push_back(part_id);
                      roofparts_lr_attributes.sub_terminal("dak_deel_id").push_back(roofpart_i++);
                      if (semobject.contains("b3_azimut")) {
                        if (!roofparts_lr_attributes.has_sub_terminal("b3_azimut")) {
                          roofparts_lr_attributes.add_vector("b3_azimut", typeid(float));
                        }
                        if (semobject["b3_azimut"].is_null()) {
                          roofparts_lr_attributes.sub_terminal("b3_azimut").push_back_any(std::any());
                        } else {
                          roofparts_lr_attributes.sub_terminal("b3_azimut").push_back(semobject["b3_azimut"].get<float>());
                        }
                      }
                      if (semobject.contains("b3_hellingshoek")) {
                        if (!roofparts_lr_attributes.has_sub_terminal("b3_hellingshoek")) {
                          roofparts_lr_attributes.add_vector("b3_hellingshoek", typeid(float));
                        }
                        if (semobject["b3_hellingshoek"].is_null()) {
                          roofparts_lr_attributes.sub_terminal("b3_hellingshoek").push_back_any(std::any());
                        } else {
                          roofparts_lr_attributes.sub_terminal("b3_hellingshoek").push_back(semobject["b3_hellingshoek"].get<float>());
                        }
                      }
                      if (semobject.contains("b3_h_dak_50p")) {
                        if (!roofparts_lr_attributes.has_sub_terminal("b3_h_dak_50p")) {
                          roofparts_lr_attributes.add_vector("b3_h_dak_50p", typeid(float));
                        }
                        if (semobject["b3_h_dak_50p"].is_null()) {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_50p").push_back_any(std::any());
                        } else {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_50p").push_back(semobject["b3_h_dak_50p"].get<float>());
                        }
                      }
                      if (semobject.contains("b3_h_dak_70p")) {
                        if (!roofparts_lr_attributes.has_sub_terminal("b3_h_dak_70p")) {
                          roofparts_lr_attributes.add_vector("b3_h_dak_70p", typeid(float));
                        }
                        if (semobject["b3_h_dak_70p"].is_null()) {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_70p").push_back_any(std::any());
                        } else {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_70p").push_back(semobject["b3_h_dak_70p"].get<float>());
                        }
                      }
                      if (semobject.contains("b3_h_dak_min")) {
                        if (!roofparts_lr_attributes.has_sub_terminal("b3_h_dak_min")) {
                          roofparts_lr_attributes.add_vector("b3_h_dak_min", typeid(float));
                        }
                        if (semobject["b3_h_dak_min"].is_null()) {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_min").push_back_any(std::any());
                        } else {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_min").push_back(semobject["b3_h_dak_min"].get<float>());
                        }
                      }
                      if (semobject.contains("b3_h_dak_max")) {
                        if (!roofparts_lr_attributes.has_sub_terminal("b3_h_dak_max")) {
                          roofparts_lr_attributes.add_vector("b3_h_dak_max", typeid(float));
                        }
                        if (semobject["b3_h_dak_max"].is_null()) {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_max").push_back_any(std::any());
                        } else {
                          roofparts_lr_attributes.sub_terminal("b3_h_dak_max").push_back(semobject["b3_h_dak_max"].get<float>());
                        }
                      }

                    }

                    mesh.push_polygon(ring, st_map[semobject["type"].get<std::string>()]);
                  }
                  roofparts.push_back(roofparts_);
                  meshes.push_back(mesh);
                  meshes_attributes.sub_terminal("identificatie").push_back(identificatie);
                  meshes_attributes.sub_terminal("pand_deel_id").push_back(part_id);
                  n_mesh++;
                }
              }
            }
            //
            if (!optimal_lod_geometry_found) {
              meshes.push_back(Mesh());
              meshes_attributes.sub_terminal("identificatie").push_back(identificatie);
              meshes_attributes.sub_terminal("pand_deel_id").push_back(part_id);
            }
          }
        }
        if(n_attr!=n_mesh) {
          std::cout << "Pushing attr n=" <<n_attr<<" and mesh n=" <<n_mesh <<std::endl;
          std::cout << featurestr << std::endl;
        }
      }
    } else { // not 3dbag_buildings_mode
      // create attributes from attribute_spec
      for(auto& [name,type] : attribute_filter_map) {
        if(type=="float")
          attributes.add_vector(name, typeid(float));
        else if(type=="string")
          attributes.add_vector(name, typeid(std::string));
        else if(type=="int")
          attributes.add_vector(name, typeid(int));
        else if(type=="bool")
          attributes.add_vector(name, typeid(bool));
        else
          throw(gfException("Illegal type in attribute_spec string: "+manager.substitute_globals(atribute_spec)));
      }

      for (size_t i=0; i< features_inp.size(); ++i) {
        // std::cout<< "FI:" << i<< std::endl;
        auto& featurestr = features_inp.get<std::string>(i);
        if (featurestr.size()==0) {
          std::cout << "empty feature string for feature; skipping...\n";
          continue;
        }
        // std::cout<< featurestr << std::endl;
        nlohmann::json feature;
        try {
          feature = nlohmann::json::parse(featurestr);
        } catch (const std::exception& e) {
          throw(gfIOError(e.what()));
        }

        if(feature["type"] != "CityJSONFeature") {
          throw(gfException("input is not CityJSONFeature"));
        }
        size_t n_attr=0, n_mesh=0;
        std::vector<std::vector<double>> vertices = feature["vertices"];
        // we can only push once the attributes per CityObject
        auto pushed_attributes = false;
        for( auto [id, cobject] : feature["CityObjects"].items() ) {

          auto ftype = cobject["type"].get<std::string>();
          std::cout<< "type:" << ftype <<std::endl;

          if (filter_by_ftype) if(!feature_filter.count(ftype)) {
            std::cout<< "skipping..." <<std::endl;
            continue;
          }

          std::string selected_lod="";
          if(lod_filter.count(ftype)) {
            selected_lod = manager.substitute_globals(lod_filter[ftype]);
          }
          if(selected_lod.size()==0) {
            float max_lodf = 0;
            std::cout<< "\navailable lod: ";
            for (const auto& geom : cobject["geometry"]) {
              auto lod = geom["lod"].get<std::string>();
              std::cout<<lod << ", ";
              auto lodf = std::stof(lod);
              if(lodf > max_lodf) {
                max_lodf = lodf;
                selected_lod = lod;
              }
            }
          }
          std::cout<< "selected lod: " << selected_lod << std::endl;

          // std::cout<< "CID:" << id << std::endl;
          // std::cout<< "vertex_count:" << cobject[]<< std::endl;
          // get_attributes
          bool pushed_geometry = false;
          for (const auto& geom : cobject["geometry"]) {
            // get geometry for highest lod
            std::cout << "found geom with lod "<< geom["lod"] << std::endl;
            if(geom["lod"] != selected_lod) continue;
            if (geom["type"] == "Solid") {
              Mesh mesh;
              // get faces of exterior shell (interior ones ignored)
              for (const auto& ext_face : geom["boundaries"][0]) {
                LinearRing ring;
                for (const auto& i : ext_face[0]) { // get vertices of outer rings
                  ring.push_back(
                    manager.coord_transform_fwd(
                      float((vertices[i][0] * jscale[0])+jtranslate[0]),
                      float((vertices[i][1] * jscale[1])+jtranslate[1]),
                      float((vertices[i][2] * jscale[2])+jtranslate[2])
                    )
                  );
                  // get the surface type
                }
                mesh.push_polygon(ring, 2);
              }
              meshes.push_back(mesh);
              pushed_geometry = true;
            } else if (geom["type"] == "MultiSurface") {
              Mesh mesh;
              // get faces of exterior shell
              for (const auto& ext_face : geom["boundaries"]) {
                LinearRing ring;
                for (const auto& i : ext_face[0]) { // get vertices of outer rings
                  ring.push_back(
                    manager.coord_transform_fwd(
                      float((vertices[i][0] * jscale[0])+jtranslate[0]),
                      float((vertices[i][1] * jscale[1])+jtranslate[1]),
                      float((vertices[i][2] * jscale[2])+jtranslate[2])
                    )
                  );
                  // get the surface type
                }
                mesh.push_polygon(ring, 2);
              }
              meshes.push_back(mesh);
              pushed_geometry = true;
            } else {
              throw(gfIOError("Unsupported geometry type"));
            }
          }
          if (pushed_geometry && !pushed_attributes) {
            auto& jattributes = cobject["attributes"];
            if (use_parent_attributes_) {
                //check if the feature has a parent
                if(cobject["parents"].size() > 0){
                    auto& ref = cobject["parents"][0];
                    jattributes = feature["CityObjects"][ ref ] ["attributes"];
                }
            }

            pushed_attributes = true;
            vector_output("feature_type").push_back(ftype);
            for(auto& [name, attribute] : attributes.sub_terminals()) {
              if(!jattributes.count(name)) {
                attribute->push_back_any(std::any());
                continue;
              }
              auto& jval = jattributes[name];
              if (jval.is_null()) {
                attribute->push_back_any(std::any());
                continue;
              }
              if (attribute->accepts_type( typeid(float)) ) {
                if (jval.is_number_float())
                  attribute->push_back(jval.get<float>());
                else {
                  try {
                    const float jval_float{ std::stof(jval.get<std::string>()) };
                    attribute->push_back(jval_float);
                  } catch (std::invalid_argument const& ex) {
                    std::cout << "could not convert attribute " << name
                              << " from " << jval.type_name() << " to float"
                              << std::endl;
                  } catch (std::out_of_range const& ex) {
                    std::cout << "attribute value (" << jval.get<std::string>()
                              << ") of " << name
                              << " is out of range for a float" << std::endl;
                  }
                }
              } else if (attribute->accepts_type( typeid(int) )) {
                if (jval.is_number_integer())
                  attribute->push_back(jval.get<int>());
                else {
                  try {
                    const int jval_int{ std::stoi(jval.get<std::string>()) };
                    attribute->push_back(jval_int);
                  } catch (std::invalid_argument const& ex) {
                    std::cout << "could not convert attribute " << name
                              << " from " << jval.type_name() << " to int"
                              << std::endl;
                  } catch (std::out_of_range const& ex) {
                    std::cout << "attribute value (" << jval.get<std::string>()
                              << ") of " << name << " is out of range for an int"
                              << std::endl;
                  }
                }
              } else if (attribute->accepts_type( typeid(bool) )) {
                if (jval.is_boolean())
                  attribute->push_back(jval.get<bool>());
                else {
                  bool b;
                  std::istringstream(jval.get<std::string>()) >> std::boolalpha >>
                    b;
                  attribute->push_back(b);
                }
              } else if (attribute->accepts_type( typeid(std::string) )) {
                attribute->push_back(jval.get<std::string>());
              }
            }
          }
        }
      }
    }


  }


} // namespace geoflow::nodes::basic3d
