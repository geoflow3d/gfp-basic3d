
#include "nodes.hpp"
#include <filesystem>
#include <geoflow/nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace geoflow::nodes::basic3d
{
  static std::unordered_map <std::string, int> st_map = 
  {
    {"RoofSurface", 0},
    {"GroundSurface",1},
    {"WallSurface", 2},
    {"ClosureSurface", 3},
    {"OuterCeilingSurface", 4},
    {"OuterFloorSurface", 5},
    {"Window", 6},
    {"Door", 7}
  };

  void CityJSONReaderNode::process() {
    // get filepath from paramter

    // read json file from disk
    std::ifstream inputStream(filepath_);
    nlohmann::json json;
    try {
      inputStream >> json;
    } catch (const std::exception& e) {
      std::cerr << e.what();
      return;
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

  void add_vertices_polygon(std::map<arr3f, size_t>& vertex_map, std::vector<arr3f>& vertex_vec, std::set<arr3f>& vertex_set, const LinearRing& polygon) {
    size_t v_cntr = vertex_vec.size();
    for (auto &vertex : polygon)
    {
      auto [it, did_insert] = vertex_set.insert(vertex);
      if (did_insert)
      {
        vertex_map[vertex] = v_cntr++;
        vertex_vec.push_back(vertex);
      }
    }
  }

  void add_vertices_mesh(std::map<arr3f, size_t>& vertex_map, std::vector<arr3f>& vertex_vec, std::set<arr3f>& vertex_set, const Mesh& mesh) {
    for (auto &face : mesh.get_polygons())
    {
      add_vertices_polygon(vertex_map, vertex_vec, vertex_set, face);
    }
  }

  std::vector<std::vector<size_t>> CityJSONWriterNode::LinearRing2jboundary(std::map<arr3f, size_t>& vertex_map, const LinearRing& face) {
    std::vector<std::vector<size_t>> jface;
    std::vector<size_t> exterior_ring;
    for (auto &vertex : face) {
      exterior_ring.push_back(vertex_map[vertex]);
    }
    jface.emplace_back(std::move(exterior_ring));
    for (auto &iring : face.interior_rings()) {
      std::vector<size_t> interior_ring;
      for (auto &vertex : iring) {
        interior_ring.push_back(vertex_map[vertex]);
      }
      jface.emplace_back(std::move(interior_ring));
    }
    return jface;
  }

  nlohmann::json::object_t CityJSONWriterNode::mesh2jSolid(const Mesh& mesh, const char* lod, std::map<arr3f, size_t>& vertex_map) {
    auto geometry = nlohmann::json::object();
    geometry["type"] = "Solid";
    if(version_1_0_) {
      geometry["lod"] = atof(lod);
    } else {
      geometry["lod"] = lod;
    };
    std::vector<std::vector<std::vector<size_t>>> exterior_shell;

    for (auto &face : mesh.get_polygons())
    {
      exterior_shell.emplace_back( LinearRing2jboundary(vertex_map, face) );
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

  void CityJSONWriterNode::process() {
    // inputs
    auto& footprints = vector_input("footprints");

    nlohmann::json outputJSON;

    outputJSON["type"] = "CityJSON";
    if (version_1_0_) {
      outputJSON["version"] = "1.0";
    } else {
      outputJSON["version"] = "1.1";
    };
    outputJSON["CityObjects"] = nlohmann::json::object();

    std::map<arr3f, size_t> vertex_map;
    std::vector<arr3f> vertex_vec;
    std::set<arr3f> vertex_set;
    size_t id_cntr = 0;
    size_t bp_counter = 0;
    std::string identifier_attribute = manager.substitute_globals(identifier_attribute_);

    auto& multisolids_lod12 = vector_input("geometry_lod12");
    auto& multisolids_lod13 = vector_input("geometry_lod13");
    auto& multisolids_lod22 = vector_input("geometry_lod22");
    for (size_t i=0; i<multisolids_lod22.size(); ++i) {
      auto building = nlohmann::json::object();
      auto b_id = std::to_string(++id_cntr);
      building["type"] = "Building";
      // building["attributes"]
      // building["children"]

      // Building atributes
      bool id_from_attr = false;
      auto jattributes = nlohmann::json::object();
      for (auto& term : poly_input("attributes").sub_terminals()) {
        if (!term->get_data_vec()[i].has_value()) continue;
        auto tname = term->get_name();
        
        //see if we need to rename this attribute
        auto search = output_attribute_names.find(tname);
        if(search != output_attribute_names.end()) {
          //ignore if the new name is an empty string
          if(search->second.size()!=0)
            tname = search->second;
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
        }
      }

      building["attributes"] = jattributes;
      
      // footprint geometry
      auto fp_geometry = nlohmann::json::object();
      if (version_1_0_) { 
        fp_geometry["lod"] = 0;
      } else {
        fp_geometry["lod"] = "0";
      }
      fp_geometry["type"] = "MultiSurface";

      auto& footprint = footprints.get<LinearRing>(i);
      add_vertices_polygon(vertex_map, vertex_vec, vertex_set, footprint);
      fp_geometry["boundaries"] = {LinearRing2jboundary(vertex_map, footprint)};
      building["geometry"].push_back(fp_geometry);
      // building["geometry"] = nlohmann::json::array();

      std::vector<std::string> buildingPartIds;

      bool has_solids = multisolids_lod22.get_data_vec()[i].has_value();
      if (has_solids) {
        // geometries
        const auto& solids_lod12 = multisolids_lod12.get<std::unordered_map<int, Mesh>>(i);
        const auto& solids_lod13 = multisolids_lod13.get<std::unordered_map<int, Mesh>>(i);

        for ( const auto& [sid, solid_lod22] : multisolids_lod22.get<std::unordered_map<int, Mesh>>(i) ) {
          auto buildingPart = nlohmann::json::object();
          auto bp_id = b_id + "-" + std::to_string(sid);
          
          buildingPartIds.push_back(bp_id);
          buildingPart["type"] = "BuildingPart";
          buildingPart["parents"] = {b_id};

          
          add_vertices_mesh(vertex_map, vertex_vec, vertex_set, solids_lod12.at(sid));
          add_vertices_mesh(vertex_map, vertex_vec, vertex_set, solids_lod13.at(sid));
          add_vertices_mesh(vertex_map, vertex_vec, vertex_set, solid_lod22);
          
          buildingPart["geometry"].push_back(mesh2jSolid(solids_lod12.at(sid), "1.2", vertex_map));
          buildingPart["geometry"].push_back(mesh2jSolid(solids_lod13.at(sid), "1.3", vertex_map));
          buildingPart["geometry"].push_back(mesh2jSolid(solid_lod22, "2.2", vertex_map));

          //attrubutes
          auto jattributes = nlohmann::json::object();
          for (auto& term : poly_input("part_attributes").sub_terminals()) {
            if (!term->get_data_vec()[i].has_value()) continue;
            auto tname = term->get_name();
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

      outputJSON["CityObjects"][b_id] = building;
    }

    Box bbox;
    bbox.add(vertex_vec);
    // auto center = bbox.center();
    std::vector<std::array<int,3>>vertices_int;
    for (auto& vertex : vertex_vec) {
      vertices_int.push_back({ 
        int( vertex[0] * 1000 ),
        int( vertex[1] * 1000 ),
        int( vertex[2] * 1000 )
      });
    }
    outputJSON["vertices"] = vertices_int;
    outputJSON["transform"] = {
      {"translate", *manager.data_offset},
      {"scale", {0.001, 0.001, 0.001}}
    };

    // metadata
    auto metadata = nlohmann::json::object();
    auto minp = bbox.min();
    auto maxp = bbox.max();
    metadata["geographicalExtent"] = {
      minp[0]+(*manager.data_offset)[0],
      minp[1]+(*manager.data_offset)[1],
      minp[2]+(*manager.data_offset)[2],
      maxp[0]+(*manager.data_offset)[0],
      maxp[1]+(*manager.data_offset)[1],
      maxp[2]+(*manager.data_offset)[2]
    };

    // TODO create node parameters for these
    metadata["referenceSystem"] = manager.substitute_globals(referenceSystem_);
    metadata["citymodelIdentifier"] = manager.substitute_globals(citymodelIdentifier_);
    metadata["datasetTitle"] = manager.substitute_globals(datasetTitle_);
    metadata["datasetReferenceDate"] = manager.substitute_globals(datasetReferenceDate_);
    metadata["geographicLocation"] = manager.substitute_globals(geographicLocation_);
    // "metadata":{"geographicalExtent":[84372.90299658204,446339.80099951173,-1.6206239461898804,85051.81354956055,447006.0341881409,35.51251220703125],"citymodelIdentifier":"6118726d-ed69-4c62-8eb6-0b39f3a8623e","datasetReferenceDate":"2021-03-04","datasetCharacterSet":"UTF-8","datasetTopicCategory":"geoscientificInformation","distributionFormatVersion":"1.0","spatialRepresentationType":"vector","metadataStandard":"ISO 19115 - Geographic Information - Metadata","metadataStandardVersion":"ISO 19115:2014(E)","metadataCharacterSet":"UTF-8","metadataDateStamp":"2021-03-04","textures":"absent","materials":"absent","cityfeatureMetadata":{"Building":{"uniqueFeatureCount":1304,"aggregateFeatureCount":3912,"presentLoDs":{"1.2":1304,"1.3":1304,"2.2":1304}}},"presentLoDs":{"1.2":1304,"1.3":1304,"2.2":1304},"thematicModels":["Building"],"referenceSystem":"urn:ogc:def:crs:EPSG::7415","fileIdentifier":"5907.json"}

    outputJSON["metadata"] = metadata;

    auto fname = fs::path(manager.substitute_globals(filepath_));
    fs::create_directories(fname.parent_path());

    std::ofstream ofs;
    ofs.open(fname);
    ofs << std::fixed << std::setprecision(2);
    try {
      if (prettyPrint_)
        ofs << outputJSON.dump(2);
      else
        ofs << outputJSON;
    } catch (const std::exception& e) {
      std::cerr << e.what();
      return;
    }
  }
} // namespace geoflow::nodes::basic3d
