#include "nodes.hpp"
#include "CityJSONFeature_generated.h"

namespace geoflow::nodes::basic3d {

  void CityFBFeatureWriterNode::process() {
    auto& feature = input("CityJSONFeature").get<nlohmann::json>(i);
      
    if(feature["type"] != "CityJSONFeature") {
      throw(gfException("input is not CityJSONFeature"));
    }

    // Create ID
    flatbuffers::FlatBufferBuilder builder;
    auto id = builder.CreateString(feature["id"]);

    // Create vertices
    std::vector<CityFB::Vertex> verticesVec;
    for (auto&v : feature["vertices"]) {
      verticesVec.push_back(CityFB::Vertex(v[0], v[1], v[2]));
    }
    auto vertices = auto builder.CreateVectorOfStructs(verticesVec);

    // Create city objects
    for( auto& [id_, cobject] : feature["CityObjects"].items() ) {
      auto id = builder.CreateString(id_);

      // type
      CityFB::CityObjectType type;
      if(cobject["type"] == "Building") {
        type = CityFB::CityObjectType_Building;
      } else if(cobject["type"] == "BuildingPart") {
        type = CityFB::CityObjectType_BuildingPart;
      }

      // attributes

      // children
      // parent

      // geometry
      for (auto& cjgeometry : cobject["geometry"]) {
        flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<void>>> boundaries;
        auto lod = builder.CreateString(cjgeometry["lod"]);
        if(cjgeometry["type"] == "MultiSurface") {
          gRing
          gSurface = CreategSurface(
            builder,
            gRing
          )
          boundaries = CreateMultiSurface(
            builder,
            gSurface
          ).Union();
        } else if(cjgeometry["type"] == "Solid") {

        }
        geometry = CreateGeometry(
          builder,
          lod,
          nullptr, // semantics
          boundaries
        );
      }

      auto city_object = CityFB::CreateCityObject(
        builder,
        type,
        id,
        nullptr, //attributes
        CityFB::Geometry_Solid,
        geometry


      )
    }


    CityFB::CreateCityJSONFeature(
      builder,
      id,
      city_objects,
      vertices
    );
  }
}