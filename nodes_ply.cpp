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
#include "happly.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace geoflow::nodes::basic3d
{

void PLYWriterNode::process() {
  auto geometries = input("geometries").get<PointCollection>();

  happly::PLYData plyOut;

  plyOut.addElement("vertex", geometries.size());
  const size_t N = geometries.size();
  std::vector<float> xPos(N);
  std::vector<float> yPos(N);
  std::vector<float> zPos(N);
  for (size_t i = 0; i < geometries.size(); i++) {
    xPos[i] = geometries[i][0];
    yPos[i] = geometries[i][1];
    zPos[i] = geometries[i][2];
  }
  // Store
  plyOut.getElement("vertex").addProperty<float>("x", xPos);
  plyOut.getElement("vertex").addProperty<float>("y", yPos);
  plyOut.getElement("vertex").addProperty<float>("z", zPos);

  for (auto& term : poly_input("attributes").sub_terminals()) {
      vec1f fvec;
      for(size_t i=0; i<term->size(); ++i) {
        fvec.push_back( term->get<float>(i) );
      }
      plyOut.getElement("vertex").addProperty(term->get_name(), fvec);
  }

  auto fname = fs::path(manager.substitute_globals(filepath));
  fs::create_directories(fname.parent_path());

  if(write_ascii) 
    plyOut.write(fname.string(), happly::DataFormat::ASCII);
  else 
    plyOut.write(fname.string(), happly::DataFormat::Binary);

}

}