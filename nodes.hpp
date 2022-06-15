// This file is part of Geoflow
// Copyright (C) 2018-2019  Ravi Peters, 3D geoinformation TU Delft

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

#include <fstream>
#include <iomanip>
#include <filesystem>

#include <geoflow/geoflow.hpp>

namespace fs = std::filesystem;

namespace geoflow::nodes::basic3d
{

class OBJWriterNode : public Node
{
  int precision=5;
  std::string filepath;
  bool no_offset = false;

public:
  using Node::Node;
  void init()
  {
    add_input("triangles", typeid(TriangleCollection));

    add_param(ParamPath(filepath, "filepath", "File path"));
    add_param(ParamBool(no_offset, "no_offset", "Do not apply global offset"));
    add_param(ParamInt(precision, "precision", "precision"));
  }
  void process();
  bool parameters_valid() override {
    if (manager.substitute_globals(filepath).empty()) 
      return false;
    else 
      return true;
  }
};

class PLYWriterNode : public Node
{
  std::string filepath;
  bool no_offset = false;
  bool write_ascii = false;

public:
  using Node::Node;
  void init()
  {
    add_input("geometries", typeid(PointCollection));
    add_poly_input("attributes", {typeid(float)});

    add_param(ParamPath(filepath, "filepath", "File path"));
    add_param(ParamBool(no_offset, "no_offset", "Do not apply global offset"));
    add_param(ParamBool(write_ascii, "write_ascii", "Output as ascii file instead of binary"));
  }
  void process();
  bool parameters_valid() override {
    if (manager.substitute_globals(filepath).empty()) 
      return false;
    else 
      return true;
  }
};

class VecOBJWriterNode : public Node
{
  int precision=5;
  std::string filepath;
  std::string headerline_;
  std::string attribute_name = "identificatie";
  bool no_offset = false;

public:
  using Node::Node;
  void init()
  {
    add_vector_input("triangles", {typeid(TriangleCollection), typeid(MultiTriangleCollection)});
    add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string), typeid(Date), typeid(Time), typeid(DateTime)});

    add_param(ParamPath(filepath, "filepath", "File path"));
    add_param(ParamBool(no_offset, "no_offset", "Do not apply global offset"));
    add_param(ParamInt(precision, "precision", "precision"));
    add_param(ParamString(attribute_name, "attribute_name", "attribute to use as identifier for obj objects. Has to be a string attribute."));
    add_param(ParamString(headerline_, "Headerline", "add this string as a comment in the header of the OBJ file"));
  }
  void process();
  bool parameters_valid() override {
    if (manager.substitute_globals(filepath).empty()) 
      return false;
    else 
      return true;
  }
};

class CityJSONReaderNode : public Node {

  // parameter variables
  std::string filepath_;
  int extract_lod_ = 2;

  public:
  using Node::Node;
  
  void init() override {
    // declare ouput terminals
    add_vector_output("faces", typeid(LinearRing));
    add_vector_output("surface_types", typeid(int));

    // declare parameters
    add_param(ParamPath(filepath_, "filepath", "File path"));
    add_param(ParamInt(extract_lod_, "extract_lod", "precision"));
  }

  void process() override;
};

// Helper functions for processing CityJSON data
class CityJSON{

  public:
    static std::vector<std::vector<size_t>> LinearRing2jboundary(std::map<arr3f, size_t>& vertex_map, const LinearRing& face);
    static nlohmann::json::object_t mesh2jSolid(const Mesh& mesh, const char* lod, std::map<arr3f, size_t>& vertex_map);
    static void write_cityobjects(gfSingleFeatureInputTerminal& footprints,
                                  gfSingleFeatureInputTerminal& multisolids_lod12,
                                  gfSingleFeatureInputTerminal& multisolids_lod13,
                                  gfSingleFeatureInputTerminal& multisolids_lod22,
                                  gfMultiFeatureInputTerminal&  attributes,
                                  gfMultiFeatureInputTerminal&  part_attributes,
                                  json&                         outputJSON,
                                  std::vector<arr3f>&           vertex_vec,
                                  std::string&                  identifier_attribute,
                                  StrMap&                       output_attribute_names,
                                  std::array<double, 3>&        data_offset);
    static void write_to_file(const json& outputJSON, fs::path& fname, bool prettyPrint_);
    static nlohmann::json::array_t compute_geographical_extent(Box& bbox, std::array<double,3>& data_offset);
};

class CityJSONWriterNode : public Node {

  // parameter variables
  std::string filepath_;
  std::string identifier_attribute_ = "";
  std::string meta_referenceSystem_ = "https://www.opengis.net/def/crs/EPSG/0/7415";
  std::string meta_identifier_      = "42";
  std::string meta_title_           = "3D BAG development";
  std::string meta_referenceDate_   = "1970-01-01";
  std::string meta_poc_contactName_;
  std::string meta_poc_phone_;
  std::string meta_poc_address_;
  std::string meta_poc_email_;
  std::string meta_poc_type_;
  std::string meta_poc_website_;

  bool prettyPrint_ = false;

  vec1s key_options;
  StrMap output_attribute_names;

  public:
  using Node::Node;

  void init() override {
    // declare ouput terminals
    add_vector_input("footprints", typeid(LinearRing));
    add_vector_input("geometry_lod12", typeid(std::unordered_map<int, Mesh>));
    add_vector_input("geometry_lod13", typeid(std::unordered_map<int, Mesh>));
    add_vector_input("geometry_lod22", typeid(std::unordered_map<int, Mesh>));
    add_poly_input("part_attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string), typeid(Date), typeid(Time), typeid(DateTime)});
    add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string), typeid(std::string), typeid(Date), typeid(Time), typeid(DateTime)});

    // declare parameters
    add_param(ParamPath(filepath_, "filepath", "File path"));
    add_param(ParamString(identifier_attribute_, "identifier_attribute", "(Renamed) attribute to use for CityObject ID (leave empty for auto ID generation). Only works for int and string attributes."));
    add_param(ParamString(meta_identifier_, "meta_identifier", "Metadata: identifier"));
    add_param(ParamString(meta_poc_contactName_, "meta_poc_contactName", "Metadata: pointOfContact.contactName"));
    add_param(ParamString(
      meta_poc_phone_, "meta_poc_phone", "Metadata: pointOfContact.phone"));
    add_param(ParamString(meta_poc_address_, "meta_poc_address", "Metadata: pointOfContact.address"));
    add_param(ParamString(meta_poc_email_, "meta_poc_emailAddress", "Metadata: pointOfContact.emailAddress"));
    add_param(ParamString(meta_poc_type_, "meta_poc_contactType", "Metadata: pointOfContact.contactType"));
    add_param(ParamString(meta_poc_website_, "meta_poc_website", "Metadata: pointOfContact.website"));
    add_param(ParamString(meta_referenceDate_, "meta_referenceDate", "Metadata: referenceDate"));
    add_param(ParamString(meta_referenceSystem_, "meta_referenceSystem", "Metadata: referenceSystem"));
    add_param(ParamString(meta_title_, "meta_title", "Metadata: title"));
    add_param(ParamBool(prettyPrint_, "prettyPrint", "Pretty print CityJSON output"));
    add_param(ParamStrMap(output_attribute_names, key_options, "output_attribute_names", "Output attribute names"));
  }

  void on_receive(gfMultiFeatureInputTerminal& it) override {
    key_options.clear();
    if(&it == &poly_input("attributes")) {
      for(auto sub_term : it.sub_terminals()) {
        key_options.push_back(sub_term->get_full_name());
      }
    }
  };
  bool parameters_valid() override {
    if (manager.substitute_globals(filepath_).empty()) 
      return false;
    else 
      return true;
  }
  bool inputs_valid() override {
    bool has_connection = input("geometry_lod12").has_connection() ||
                          input("geometry_lod13").has_connection() ||
                          input("geometry_lod22").has_connection();

    bool has_connection_without_data = false;
    if (input("geometry_lod12").has_connection() && !input("geometry_lod12").has_data()) {
      has_connection_without_data = has_connection_without_data || true;
    } 
    if (input("geometry_lod13").has_connection() && !input("geometry_lod13").has_data()) {
      has_connection_without_data = has_connection_without_data || true;
    } 
    if (input("geometry_lod22").has_connection() && !input("geometry_lod22").has_data()) {
      has_connection_without_data = has_connection_without_data || true;
    } 

    return input("footprints").has_data() && poly_input("attributes").has_data() && !has_connection_without_data && has_connection;
  }

  void process() override;
};

class CityJSONFeatureWriterNode : public Node {

  // parameter variables
  std::string filepath_;
  std::string identifier_attribute_ = "";

  bool prettyPrint_ = false;

  vec1s key_options;
  StrMap output_attribute_names;

  float translate_x_;
  float translate_y_;
  float translate_z_;
  float scale_x_;
  float scale_y_;
  float scale_z_;

public:
  using Node::Node;

  void init() override {
    // declare ouput terminals
    add_vector_input("footprints", typeid(LinearRing));
    add_vector_input("geometry_lod12", typeid(std::unordered_map<int, Mesh>));
    add_vector_input("geometry_lod13", typeid(std::unordered_map<int, Mesh>));
    add_vector_input("geometry_lod22", typeid(std::unordered_map<int, Mesh>));
    add_poly_input("part_attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string), typeid(Date), typeid(Time), typeid(DateTime)});
    add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string), typeid(std::string), typeid(Date), typeid(Time), typeid(DateTime)});

    // declare parameters
    add_param(ParamPath(filepath_, "filepath", "File path"));
    add_param(ParamString(identifier_attribute_, "identifier_attribute", "(Renamed) attribute to use for CityObject ID (leave empty for auto ID generation). Only works for int and string attributes."));
    add_param(ParamBool(prettyPrint_, "prettyPrint", "Pretty print CityJSON output"));
    add_param(ParamStrMap(output_attribute_names, key_options, "output_attribute_names", "Output attribute names"));
    add_param(ParamFloat(translate_x_, "translate_x", "CityJSON transform.translate.x"));
    add_param(ParamFloat(translate_y_, "translate_y", "CityJSON transform.translate.y"));
    add_param(ParamFloat(translate_z_, "translate_z", "CityJSON transform.translate.z"));
    add_param(ParamFloat(scale_x_, "scale_x", "CityJSON transform.scale.x"));
    add_param(ParamFloat(scale_y_, "scale_y", "CityJSON transform.scale.y"));
    add_param(ParamFloat(scale_z_, "scale_z", "CityJSON transform.scale.z"));
  }

  void on_receive(gfMultiFeatureInputTerminal& it) override {
    key_options.clear();
    if(&it == &poly_input("attributes")) {
      for(auto sub_term : it.sub_terminals()) {
        key_options.push_back(sub_term->get_full_name());
      }
    }
  };

  bool parameters_valid() override {
    if (manager.substitute_globals(filepath_).empty()) 
      return false;
    else 
      return true;
  }

  bool inputs_valid() override {
    bool has_connection = input("geometry_lod12").has_connection() ||
                          input("geometry_lod13").has_connection() ||
                          input("geometry_lod22").has_connection();

    bool has_connection_without_data = false;
    if (input("geometry_lod12").has_connection() && !input("geometry_lod12").has_data()) {
      has_connection_without_data = has_connection_without_data || true;
    }
    if (input("geometry_lod13").has_connection() && !input("geometry_lod13").has_data()) {
      has_connection_without_data = has_connection_without_data || true;
    }
    if (input("geometry_lod22").has_connection() && !input("geometry_lod22").has_data()) {
      has_connection_without_data = has_connection_without_data || true;
    }

    return input("footprints").has_data() && poly_input("attributes").has_data() && !has_connection_without_data && has_connection;
  }

  void process() override;
};

} // namespace geoflow::nodes::basic3d