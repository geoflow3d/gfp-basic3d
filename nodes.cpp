#include "nodes.hpp"
#include <iomanip>

namespace geoflow::nodes::basic3d
{

void RingTriangulatorNode::process()
{
  auto &rings = vector_input("rings");
  const auto &values_in = input("valuesf").get<vec1f>();
  typedef uint32_t N;

  TriangleCollection triangles;
  vec3f normals;
  vec1f values_out;
  size_t vi = 0;
  for (size_t i = 0; i < rings.size(); ++i)
  {
    auto poly_3d = rings.get<LinearRing>(i);
    std::cout << poly_3d[0][0] << poly_3d[0][1] << poly_3d[0][2] << "\n";
    auto normal = calculate_normal(poly_3d);
    vec2f poly_2d = project_ring_2d(poly_3d, normal);
    std::cout << poly_3d[0][0] << poly_3d[0][1] << poly_3d[0][2] << "\n";
    vec3f vertices;
    std::vector<N> indices = mapbox::earcut<N>(std::vector<vec2f>({poly_2d}));
    for (auto i : indices)
    {
      vertices.push_back({poly_3d[i]});
    }
    for (size_t i = 0; i < indices.size() / 3; ++i)
    {
      Triangle triangle;
      for (size_t j = 0; j < 3; ++j)
      {
        triangle[j] = {vertices[i * 3 + j][0], vertices[i * 3 + j][1], vertices[i * 3 + j][2]};
        normals.push_back({normal.x, normal.y, normal.z});
        values_out.push_back(values_in[vi]);
      }
      triangles.push_back(triangle);
    }
    vi++;
  }

  // set outputs
  output("triangles").set(triangles);
  output("normals").set(normals);
  output("valuesf").set(values_out);
}

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
  ofs.open(filepath);
  ofs << std::fixed << std::setprecision(3);
  for (auto &v : vertex_vec)
  {
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
  ofs.open(filepath);
  ofs << std::fixed << std::setprecision(3);
  for (auto &v : vertex_vec)
  {
    if (no_offset)
      ofs << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    else
      ofs << "v " << v[0] + (*manager.data_offset)[0] << " " << v[1] + (*manager.data_offset)[1] << " " << v[2] + (*manager.data_offset)[2] << "\n";
  }
  for (size_t j = 0; j < triangles.size(); ++j)
  {
    ofs << "o " << j << "\n";
    for (auto &triangle : triangles.get<TriangleCollection>(j))
    {
      ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << "\n";
    }
  }
  ofs.close();
}

} // namespace geoflow::nodes::basic3d
