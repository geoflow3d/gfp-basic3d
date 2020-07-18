#include "nodes.hpp"
#include <iomanip>

namespace geoflow::nodes::basic3d
{
void OBJWriterNode::process()
{
  //auto& t_in = vector_input("triangles");
  //std::vector<Triangle> triangles;
  //for (size_t i = 0; i < t_in.size(); ++i) {
  //  triangles.push_back(t_in.get<Triangle>(i));
  //}

  auto &triangles = input("triangles").get<TriangleCollection>();

  std::map<arr3f, size_t> vertex_map;
  std::vector<arr3f> vertex_vec;
  {
    size_t v_cntr = 1;
    std::set<arr3f> vertex_set;
    for (auto &triangle : triangles)
    {
      for (auto &vertex : triangle)
      {
        auto [it, did_insert] = vertex_set.insert(vertex);
        if (did_insert)
        {
          vertex_map[vertex] = v_cntr++;
          vertex_vec.push_back(vertex);
        }
      }
    }
  }
  std::ofstream ofs;
  ofs.open(manager.substitute_globals(filepath));
  ofs << std::fixed << std::setprecision(precision);
  for (auto &v : vertex_vec)
  {
    if (no_offset)
      ofs << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    else
      ofs << "v " << v[0] + (*manager.data_offset)[0] << " " << v[1] + (*manager.data_offset)[1] << " " << v[2] + (*manager.data_offset)[2] << "\n";
  }
  for (auto &triangle : triangles)
  {
    ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << std::endl;
  }
  ofs.close();
}

void VecOBJWriterNode::process()
{
  auto &triangles = vector_input("triangles");

  std::map<arr3f, size_t> vertex_map;
  std::vector<arr3f> vertex_vec;
  {
    size_t v_cntr = 1;
    std::set<arr3f> vertex_set;
    for (size_t j = 0; j < triangles.size(); ++j)
    {
      if(!triangles.get_data_vec()[j].has_value()) continue;
      for (auto &triangle : triangles.get<TriangleCollection>(j))
      {
        for (auto &vertex : triangle)
        {
          auto [it, did_insert] = vertex_set.insert(vertex);
          if (did_insert)
          {
            vertex_map[vertex] = v_cntr++;
            vertex_vec.push_back(vertex);
          }
        }
      }
    }
  }
  std::ofstream ofs;
  ofs.open(manager.substitute_globals(filepath));
  ofs << std::fixed << std::setprecision(precision);
  for (auto &v : vertex_vec)
  {
    if (no_offset)
      ofs << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    else
      ofs << "v " << v[0] + (*manager.data_offset)[0] << " " << v[1] + (*manager.data_offset)[1] << " " << v[2] + (*manager.data_offset)[2] << "\n";
  }
  for (size_t j = 0; j < triangles.size(); ++j)
  {
    if(!triangles.get_data_vec()[j].has_value()) continue;
    ofs << "o " << j << "\n";
    for (auto &triangle : triangles.get<TriangleCollection>(j))
    {
      ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << "\n";
    }
  }
  ofs.close();
}

} // namespace geoflow::nodes::basic3d
