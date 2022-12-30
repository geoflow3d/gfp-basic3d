#include "nodes.hpp"
#include "CityJSONFeature_generated.h"

namespace geoflow::nodes::basic3d {

  std::vector<flatbuffers::Offset<CityFB::Surface>> create_surfaces_vector(flatbuffers::FlatBufferBuilder& builder, const nlohmann::json& json) {
    std::vector<flatbuffers::Offset<CityFB::Surface>> surfaces_vector;
    for(auto& cjsurface : json) {
      std::vector<flatbuffers::Offset<CityFB::Ring>> rings_vector;
      for(auto& cjring : cjsurface) {
        std::vector<uint32_t> indices_vector;
        for(auto& idx : cjring) {
          indices_vector.push_back(idx);
        }
        rings_vector.push_back(
          CityFB::CreateRing(builder, 
            builder.CreateVector(indices_vector)
          )
        );
      }
      surfaces_vector.push_back(
        CityFB::CreateSurface(builder, 
          builder.CreateVector(rings_vector)
        )
      );
    }
    return surfaces_vector;
  }

  std::vector<flatbuffers::Offset<CityFB::Shell>> create_shells_vector(flatbuffers::FlatBufferBuilder& builder, const nlohmann::json& json) {
    std::vector<flatbuffers::Offset<CityFB::Shell>> shells_vector;
    for(auto& cjshell : json) {
      auto surfaces_vector = create_surfaces_vector(builder, cjshell);
      shells_vector.push_back(
        CityFB::CreateShell(builder, 
          builder.CreateVector(surfaces_vector)
        )
      );
    };
    return shells_vector;
  }

  void CityFBFeatureWriterNode::process() {
    auto& feature = input("CityJSONFeature").get<nlohmann::json>();
      
    if(feature["type"] != "CityJSONFeature") {
      throw(gfException("input is not CityJSONFeature"));
    }

    // Create ID
    flatbuffers::FlatBufferBuilder builder;
    auto id = builder.CreateString(feature["id"].get<std::string>());

    // Create vertices
    std::vector<CityFB::Vertex> vertices_vector;
    for (auto&v : feature["vertices"]) {
      vertices_vector.push_back(
        CityFB::Vertex(v[0], v[1], v[2])
      );
    }

    // Create city objects
    std::vector<flatbuffers::Offset<CityFB::CityObject>> city_objects_vector;
    for( auto& [id_, cobject] : feature["CityObjects"].items() ) {
      

      // type
      CityFB::CityObjectType type;
      if(cobject["type"] == "Building") {
        type = CityFB::CityObjectType_Building;
      } else if(cobject["type"] == "BuildingPart") {
        type = CityFB::CityObjectType_BuildingPart;
      }

      // id
      auto id = builder.CreateString(id_);

      // attributes


      // geometry
      std::vector<flatbuffers::Offset<CityFB::Geometry>> geometries_vector;
      for (auto& cjgeometry : cobject["geometry"]) {
        CityFB::Boundaries boundaries_type;
        flatbuffers::Offset<void> boundaries;

        auto lod = builder.CreateString(cjgeometry["lod"].get<std::string>());
        if(cjgeometry["type"] == "MultiSurface") {
          auto surfaces_vector = create_surfaces_vector(builder, cjgeometry["boundaries"]);
          boundaries_type = CityFB::Boundaries_MultiSurface;
          boundaries = CityFB::CreateMultiSurface(builder,
            builder.CreateVector(surfaces_vector)
          ).Union();

        } else if(cjgeometry["type"] == "Solid") {
          auto shells_vector = create_shells_vector(builder, cjgeometry["boundaries"]);
          boundaries_type = CityFB::Boundaries_Solid;
          boundaries = CityFB::CreateSolid(builder,
            builder.CreateVector(shells_vector)
          ).Union();
        }
        geometries_vector.push_back(
          CityFB::CreateGeometry(
            builder,
            lod,
            boundaries_type,
            boundaries,
            0 // semantics
          )
        );
      }

      // children

      // parent
      

      city_objects_vector.push_back(
        CityFB::CreateCityObject(
          builder,
          type,
          id,
          0, //attributes
          builder.CreateVector(geometries_vector),
          0, //children
          0 //parent
        )
      );
    }


    CityFB::CreateCityJSONFeature(
      builder,
      id,
      builder.CreateVector(city_objects_vector),
      builder.CreateVectorOfStructs(vertices_vector)
    );
  }
}