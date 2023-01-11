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


namespace geoflow::nodes::basic3d
{
  GLTFWriterNode::process() {
    auto& triangle_collections = input_vector("triangles");
    auto& normals = input_vector("normals");

    for(size_t i=0; i< triangle_collections.size(); ++i) {
      auto& tc = triangle_collections.get<TriangleCollection>(i);
      auto& n = normals.get<vec3f>(i);
      tc[0].data(), tc.vertex_count(), tc.dimension()
      n[0].data()
    }
  }
}