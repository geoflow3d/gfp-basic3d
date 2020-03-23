#include "nodes.hpp"
#include <iomanip>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <poly2tri/poly2tri.h>

namespace geoflow::nodes::basic3d
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

std::vector<std::vector<p2t::Point*>> project_polygon_2d(const LinearRing poly, const glm::vec3 &n)
{
  std::vector<std::vector<p2t::Point*>> poly_2d;
  auto nx = glm::normalize(glm::vec3(-n.y, n.x, 0));
  auto ny = -glm::normalize(glm::cross(nx, n));
  // no need to project when normal is perfectly pointing up
  // mirror faces that point downwards
  bool do_not_project = false;
  if (std::fabs(n.y)<1E-5 && std::fabs(n.x)<1E-5) {
    if (n.z > 0) {
      nx = glm::vec3(1,0,0);
      ny = glm::vec3(0,1,0);
    } else {
      nx = glm::vec3(1,0,0);
      ny = -glm::vec3(0,1,0);
    }
  }
  // auto up = glm::vec3(0,0,1);
  // auto angle = glm::angle(n, up);
  // auto axis = glm::cross(n, up);
  // auto R = glm::rotate(angle, axis);
  std::vector<p2t::Point*> outer;
  size_t index=0;
  for (const auto &p : poly)
  {
    // auto pp = R * glm::vec4(p[0], p[1], p[2], 1.0);
    p2t::Point* p2point;
    // if (do_not_project) {
    //   p2point = new p2t::Point{p[0], p[1], index++};
    // } else {
      auto pp = glm::make_vec3(p.data());
      p2point = new p2t::Point{glm::dot(pp, nx), glm::dot(pp, ny), index++};
    // }
    outer.push_back(p2point);
  }
  poly_2d.push_back(outer);
  for (const auto &ring : poly.interior_rings()) {
    std::vector<p2t::Point*> inner;
    for (const auto &p : ring)
    {
      // auto pp = R * glm::vec4(p[0], p[1], p[2], 1.0);
      p2t::Point*  p2point;
      // if (do_not_project) {
      //   p2point = new p2t::Point{p[0], p[1], index++};
      // } else {
        auto pp = glm::make_vec3(p.data());
        p2point = new p2t::Point{glm::dot(pp, nx), glm::dot(pp, ny), index++};
      // }
      inner.push_back(p2point);
    }
    poly_2d.push_back(inner);
  }
  return poly_2d;
}

void RingTriangulatorNode::process()
{
  auto &rings = vector_input("rings");
  // const auto &values_in = input("valuesf").get<vec1f>();
  typedef uint32_t N;

  TriangleCollection triangles;
  vec3f normals;
  vec1f values_out;
  vec1i ring_ids;
  size_t vi = 0;
  for (size_t ri = 0; ri < rings.size(); ++ri)
  {
    auto poly_3d = rings.get<LinearRing>(ri);
    vec3f input_vertices;
    input_vertices.insert(input_vertices.end(), poly_3d.begin(), poly_3d.end());
    for (auto& ring : poly_3d.interior_rings()) {
      input_vertices.insert(input_vertices.end(), ring.begin(), ring.end());
    }
    auto normal = calculate_normal(poly_3d);
    if (std::isnan(normal.x) || std::isnan(normal.y) || std::isnan(normal.z)){
      std::cout << "degenerate normal: " << normal[0] << " " << normal[1] << " " << normal[2] << "\n";
      continue;
    }
    auto poly_2d = project_polygon_2d(poly_3d, normal);
    p2t::CDT cdt(poly_2d[0]);
    for (size_t i=1; i<poly_2d.size(); ++i) {
      cdt.AddHole(poly_2d[i]);
    }
    cdt.Triangulate();
    // std::cout << poly_3d[0][0] << poly_3d[0][1] << poly_3d[0][2] << "\n";

    for (auto p2triangle : cdt.GetTriangles())
    {
      Triangle triangle;
      triangle = {
        input_vertices[p2triangle->GetPoint(0)->index], 
        input_vertices[p2triangle->GetPoint(1)->index], 
        input_vertices[p2triangle->GetPoint(2)->index]
      };
      for (size_t j = 0; j < 3; ++j)
      {
        normals.push_back({normal.x, normal.y, normal.z});
        // values_out.push_back(values_in[vi]);
        ring_ids.push_back(ri);
      }
      triangles.push_back(triangle);
    }
    vi++;
  }

  // set outputs
  output("triangles").set(triangles);
  output("normals").set(normals);
  output("ring_ids").set(ring_ids);
  // output("valuesf").set(values_out);
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
    ofs << "o " << j << "\n";
    for (auto &triangle : triangles.get<TriangleCollection>(j))
    {
      ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << "\n";
    }
  }
  ofs.close();
}

} // namespace geoflow::nodes::basic3d
