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
#include <ctime>
#include <iomanip>

#include <geoflow/geoflow.hpp>

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
};

class VecOBJWriterNode : public Node
{
  int precision=5;
  std::string filepath;
  std::string attribute_name = "identificatie";
  bool no_offset = false;

public:
  using Node::Node;
  void init()
  {
    add_vector_input("triangles", {typeid(TriangleCollection), typeid(MultiTriangleCollection)});
    add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string)});

    add_param(ParamPath(filepath, "filepath", "File path"));
    add_param(ParamBool(no_offset, "no_offset", "Do not apply global offset"));
    add_param(ParamInt(precision, "precision", "precision"));
    add_param(ParamString(attribute_name, "attribute_name", "attribute to use as identifier for obj objects. Has to be a string attribute."));
  }
  void process();
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

class CityJSONWriterNode : public Node {

  // parameter variables
  std::string filepath_;
  std::string identifier_attribute_ = "";
  std::string referenceSystem_ = "https://www.opengis.net/def/crs/EPSG/0/7415";
  std::string citymodelIdentifier_ = "42";
  std::string datasetTitle_ = "3D BAG development";
  std::string datasetReferenceDate_ = "1970-01-01";
  std::string geographicLocation_ = "The Netherlands";

  bool prettyPrint_ = false;
  bool version_1_0_ = true;

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
    add_poly_input("part_attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string)});
    add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string)});

    // find current date
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d");
    datasetReferenceDate_ = oss.str();

    // declare parameters
    add_param(ParamPath(filepath_, "filepath", "File path"));
    add_param(ParamString(identifier_attribute_, "identifier_attribute", "(Renamed) attribute to use for CityObject ID (leave empty for auto ID generation). Only works for int and string attributes."));
    add_param(ParamString(referenceSystem_, "referenceSystem", "referenceSystem"));
    add_param(ParamString(citymodelIdentifier_, "citymodelIdentifier", "citymodelIdentifier"));
    add_param(ParamString(datasetTitle_, "datasetTitle", "datasetTitle"));
    add_param(ParamString(datasetReferenceDate_, "datasetReferenceDate", "datasetReferenceDate"));
    add_param(ParamString(geographicLocation_, "geographicLocation", "geographicLocation"));
    add_param(ParamBool(prettyPrint_, "prettyPrint", "Pretty print CityJSON output"));
    add_param(ParamBool(version_1_0_, "version_1_0", "Output CityJSON v1.0 instead of v1.1"));
    add_param(ParamStrMap(output_attribute_names, key_options, "output_attribute_names", "Output attribute names"));
  }

  void on_receive(gfMultiFeatureInputTerminal& it) {
    key_options.clear();
    if(&it == &poly_input("attributes")) {
      for(auto sub_term : it.sub_terminals()) {
        key_options.push_back(sub_term->get_name());
      }
    }
  };

  bool inputs_valid() override {
    return 
      input("footprints").has_data() &&
      input("geometry_lod12").has_data() &&
      input("geometry_lod13").has_data() &&
      input("geometry_lod22").has_data() &&
      poly_input("attributes").has_data()
      ;
    
  }
  std::vector<std::vector<size_t>> LinearRing2jboundary(std::map<arr3f, size_t>& vertex_map, const LinearRing& face);
  nlohmann::json::object_t mesh2jSolid(const Mesh& mesh, const char* lod, std::map<arr3f, size_t>& vertex_map);

  void process() override;
};

} // namespace geoflow::nodes::basic3d