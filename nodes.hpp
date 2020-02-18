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

class RingTriangulatorNode : public Node
{
public:
  using Node::Node;

  void init()
  {
    // declare ouput terminals
    add_vector_input("rings", typeid(LinearRing));
    // add_input("valuesf", typeid(vec1f));
    // add_output("valuesf", typeid(vec1f));
    add_output("triangles", typeid(TriangleCollection), true);
    add_output("normals", typeid(vec3f), true);
  }

  void process();
};

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

    add_param("filepath", ParamPath(filepath, "File path"));
    add_param("no_offset", ParamBool(no_offset, "Do not apply global offset"));
    add_param("precision", ParamInt(precision, "precision"));
  }
  void process();
};

class VecOBJWriterNode : public Node
{
  int precision=5;
  std::string filepath;
  bool no_offset = false;

public:
  using Node::Node;
  void init()
  {
    add_vector_input("triangles", typeid(TriangleCollection));

    add_param("filepath", ParamPath(filepath, "File path"));
    add_param("no_offset", ParamBool(no_offset, "Do not apply global offset"));
    add_param("precision", ParamInt(precision, "precision"));
  }
  void process();
};
} // namespace geoflow::nodes::basic3d