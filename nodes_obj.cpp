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
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

const std::string mtl_text = 
"newmtl 1\n"
"Ka 0.8700 0.2600 0.2800\n"
"Kd 0.8700 0.2600 0.2800\n"
"Ks 0.8700 0.2600 0.2800\n"
"illum 2\n"
"Ns 60.0000\n"

"newmtl 2\n"
"Ka 0.9000 0.9000 0.7500\n"
"Kd 0.9000 0.9000 0.7500\n"
"Ks 0.9000 0.9000 0.7500\n"
"illum 2\n"
"Ns 60.0000\n"

"newmtl 3\n"
"Ka 0.9000 0.9000 0.7500\n"
"Kd 0.9000 0.9000 0.7500\n"
"Ks 0.9000 0.9000 0.7500\n"
"illum 2\n"
"Ns 60.0000\n"

"newmtl 0\n"
"Ka 0.6000 0.6000 0.6000\n"
"Kd 0.6000 0.6000 0.6000\n"
"Ks 0.6000 0.6000 0.6000\n"
"illum 2\n"
"Ns 60.0000\n";

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
      ofs << "v " << v[0] + (*manager.data_offset())[0] << " " << v[1] + (*manager.data_offset())[1] << " " << v[2] + (*manager.data_offset())[2] << "\n";
  }
  for (auto &triangle : triangles)
  {
    ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << std::endl;
  }
  ofs.close();
}

void add_vertices(std::map<arr3f, size_t>& vertex_map, std::vector<arr3f>& vertex_vec, std::set<arr3f>& vertex_set, const TriangleCollection& tc) {
  size_t v_cntr = vertex_vec.size();
  for (auto &triangle : tc)
  {
    for (auto &vertex : triangle)
    {
      auto [it, did_insert] = vertex_set.insert(vertex);
      if (did_insert)
      {
        vertex_map[vertex] = ++v_cntr;
        vertex_vec.push_back(vertex);
      }
    }
  }
}

void write_triangles(const TriangleCollection& tc, std::ofstream& ofs, std::map<arr3f, size_t>& vertex_map) {
  for (auto &triangle : tc)
  {
    ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << "\n";
  }
}

void write_triangles(const TriangleCollection& tc, const std::vector<attribute_value>& attr, std::ofstream& ofs, std::map<arr3f, size_t>& vertex_map) {
  for (size_t i=0; i< tc.size(); ++i)
  {
    auto& triangle = tc[i];
    auto& label = std::get<int>(attr[i]);
    ofs << "usemtl " << label << "\n";
    ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << "\n";
  }
}

void VecOBJWriterNode::process()
{
  auto &triangles = vector_input("triangles");

  const gfSingleFeatureOutputTerminal* id_term;
  auto id_attr_name = manager.substitute_globals(attribute_name);
  bool use_id_from_attribute = false;
  for (auto& term : poly_input("attributes").sub_terminals()) {
    if ( term->get_name() == id_attr_name && term->accepts_type(typeid(std::string)) ) {
      id_term = term;
      use_id_from_attribute = true;
    }
  }
  
  std::map<arr3f, size_t> vertex_map;
  std::vector<arr3f> vertex_vec;
  
  if(triangles.is_connected_type(typeid(TriangleCollection))) {
    {
      std::set<arr3f> vertex_set;
      for (size_t j = 0; j < triangles.size(); ++j)
      {
        if(!triangles.get_data_vec()[j].has_value()) continue;
        const auto& tc = triangles.get<TriangleCollection>(j);
        add_vertices(vertex_map, vertex_vec, vertex_set, tc);
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
        ofs << "v " << v[0] + (*manager.data_offset())[0] << " " << v[1] + (*manager.data_offset())[1] << " " << v[2] + (*manager.data_offset())[2] << "\n";
    }
    for (size_t j = 0; j < triangles.size(); ++j)
    {
      if(!triangles.get_data_vec()[j].has_value()) continue;
      ofs << "o " << j << "\n";
      write_triangles(triangles.get<TriangleCollection>(j), ofs, vertex_map);
    }
    ofs.close();
  } else if(triangles.is_connected_type(typeid(MultiTriangleCollection))) {

    {
      std::set<arr3f> vertex_set;
      for (size_t j = 0; j < triangles.size(); ++j)
      {
        if(!triangles.get_data_vec()[j].has_value()) continue;
        
        auto& mtcs = triangles.get<MultiTriangleCollection>(j);
        for(size_t i=0; i<mtcs.tri_size(); i++) {
          const auto& tc = mtcs.tri_at(i);
          add_vertices(vertex_map, vertex_vec, vertex_set, tc);
        }
      }
    }
    auto fname = manager.substitute_globals(filepath);
    fname = substitute_from_term(fname, poly_input("attributes"));
    auto mtl_path = fs::path(fname+".mtl");

    fs::create_directories(mtl_path.parent_path());

    std::cout << "writing to " << fname << std::endl;
    
    std::ofstream ofs_mtl;
    ofs_mtl.open(mtl_path.c_str());
    ofs_mtl << mtl_text;
    ofs_mtl.close();
    
    std::ofstream ofs;
    ofs.open(fname);
    ofs << "# Created by Geoflow\n";
    ofs << "# " << manager.substitute_globals(headerline_) << "\n";
    ofs << "mtllib " << mtl_path.filename().c_str() << "\n";
    ofs << std::fixed << std::setprecision(precision);
    for (auto &v : vertex_vec)
    {
      if (no_offset)
        ofs << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
      else
        ofs << "v " << v[0] + (*manager.data_offset())[0] << " " << v[1] + (*manager.data_offset())[1] << " " << v[2] + (*manager.data_offset())[2] << "\n";
    }
    for (size_t j = 0; j < triangles.size(); ++j)
    {
      if(!triangles.get_data_vec()[j].has_value()) continue;
      if (use_id_from_attribute) {
        ofs << "o " << id_term->get<const std::string>(j) << "\n";
      } else {
        ofs << "o " << j << "\n";
      }
      auto mtcs = triangles.get<MultiTriangleCollection>(j);
      for(size_t i=0; i<mtcs.tri_size(); i++) {
          const auto& tc = mtcs.tri_at(i);
          const auto& am = mtcs.attr_at(i)["labels"];
        write_triangles(tc, am, ofs, vertex_map);
      }
    }
    ofs.close();
  }
}

} // namespace geoflow::nodes::basic3d
