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
} // namespace geoflow::nodes::basic3d