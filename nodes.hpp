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

#include "earcut.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace geoflow::nodes::basic3d
{

class RingTriangulatorNode : public Node
{
  glm::vec3 calculate_normal(const LinearRing ring)
  {
    glm::vec3 normal(0, 0, 0);
    for (size_t i = 0; i < ring.size(); ++i)
    {
      const auto &curr = ring[i];
      const auto &next = ring[(i + 1) % ring.size()];
      normal[0] += (curr[1] - next[1]) * (curr[2] + next[2]);
      normal[1] += (curr[2] - next[2]) * (curr[0] + next[0]);
      normal[2] += (curr[0] - next[0]) * (curr[1] + next[1]);
    }
    return glm::normalize(normal);
  }

  vec2f project_ring_2d(const LinearRing ring, const glm::vec3 &n)
  {
    vec2f ring_2d;
    auto nx = glm::normalize(glm::vec3(-n.y, n.x, 0));
    auto ny = glm::normalize(glm::cross(nx, n));
    for (const auto &p : ring)
    {
      auto pp = glm::make_vec3(p.data());
      ring_2d.push_back({float(glm::dot(pp, nx)),
                         float(glm::dot(pp, ny))});
    }
    return ring_2d;
  }

public:
  using Node::Node;

  void init()
  {
    // declare ouput terminals
    add_vector_input("rings", typeid(LinearRing));
    add_input("valuesf", typeid(vec1f));
    add_output("valuesf", typeid(vec1f));
    add_output("triangles", typeid(TriangleCollection));
    add_output("normals", typeid(vec3f));
  }

  void process();
};

class OBJWriterNode : public Node
{
  std::string filepath;

public:
  using Node::Node;
  void init()
  {
    add_input("triangles", typeid(TriangleCollection));

    add_param("filepath", ParamPath(filepath, "File path"));
  }
  void process();
};

class VecOBJWriterNode : public Node
{
  std::string filepath;
  bool no_offset = false;

public:
  using Node::Node;
  void init()
  {
    add_vector_input("triangles", typeid(TriangleCollection));

    add_param("filepath", ParamPath(filepath, "File path"));
    add_param("no_offset", ParamBool(no_offset, "Do not apply global offset"));
  }
  void process();
};
} // namespace geoflow::nodes::basic3d