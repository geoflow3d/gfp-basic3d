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
    plyOut.write(fname, happly::DataFormat::ASCII);
  else 
    plyOut.write(fname, happly::DataFormat::Binary);

}

}