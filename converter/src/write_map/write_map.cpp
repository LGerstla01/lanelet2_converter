#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>

#include "vector"

using namespace lanelet;
std::string tempfile(const std::string& name) {
  char tmpDir[] = "/home/gel3abt/Maps/lanelet2_example_XXXXXX";
  auto* file = mkdtemp(tmpDir);
  if (file == nullptr) {
    throw lanelet::IOError("Failed to open a temporary file for writing");
  }
  return std::string(file) + '/' + name;
}

int main(int argc, char** argv){
    
    LaneletMapPtr map;
    std::vector<std::array<float, 3>> Koords;
    std::vector<Point3d> Points;
    std::vector<std::vector<Point3d>> Lines;
    std::vector<LineString3d> Linestrings;
    std::vector<Lanelet> Lanes;

    Koords.push_back({0, 0, 0});
    Koords.push_back({50, 0, 0});
    Koords.push_back({64, 0, 0});
    Koords.push_back({200, 0, 0});

    Koords.push_back({57, 2, 0});

    Koords.push_back({0, 5, 0});
    Koords.push_back({50, 5, 0});
    Koords.push_back({64, 5, 0});
    Koords.push_back({200, 5, 0});

    Koords.push_back({52, 6, 0});
    Koords.push_back({62, 6, 0});
    Koords.push_back({56, 7, 0});
    Koords.push_back({58, 7, 0});

    Koords.push_back({0, 10, 0});
    Koords.push_back({50, 10, 0});
    Koords.push_back({52, 10, 0});
    Koords.push_back({57, 10, 0});
    Koords.push_back({62, 10, 0});
    Koords.push_back({64, 10, 0});
    Koords.push_back({200, 10, 0});

    Koords.push_back({52, 12, 0});
    Koords.push_back({57, 12, 0});
    Koords.push_back({62, 12, 0});

    Koords.push_back({52, 200, 0});
    Koords.push_back({57, 200, 0});
    Koords.push_back({62, 200, 0});

    for (int i = 0; i < Koords.size(); i++){
      Point3d p{utils::getId(), Koords[i][0], Koords[i][1], Koords[i][2]};
      //p.attributes()["ele"] = "0";
      Points.push_back(p);
    }

    Lines.push_back({Points[0], Points[1]});
    Lines.push_back({Points[1], Points[2]});
    Lines.push_back({Points[2], Points[3]});
    Lines.push_back({Points[5], Points[6]});
    Lines.push_back({Points[6], Points[7]});
    Lines.push_back({Points[7], Points[8]});
    Lines.push_back({Points[13], Points[14]});
    Lines.push_back({Points[14], Points[18]});
    Lines.push_back({Points[18], Points[19]});
    Lines.push_back({Points[20], Points[23]});
    Lines.push_back({Points[21], Points[24]});
    Lines.push_back({Points[22], Points[25]});
    Lines.push_back({Points[1], Points[4], Points[10], Points[17], Points[22]});
    Lines.push_back({Points[2], Points[4], Points[9], Points[15], Points[20]});
    Lines.push_back({Points[6], Points[11], Points[16], Points[21]});
    Lines.push_back({Points[7], Points[12], Points[16], Points[21]});
    Lines.push_back({Points[14], Points[20]});
    Lines.push_back({Points[18], Points[22]});

    for (int k = 0; k < Lines.size(); k++){
      LineString3d ls (utils::getId(), Lines[k]);
      Linestrings.push_back(ls);
    }
    
    Lanelet lane0 (utils::getId(), Linestrings[3], Linestrings[0]);
    Lanelet lane1 (utils::getId(), Linestrings[4], Linestrings[1]);
    Lanelet lane2 (utils::getId(), Linestrings[5], Linestrings[2]);
    Lanelet lane3 (utils::getId(), Linestrings[3].invert(), Linestrings[6].invert());
    Lanelet lane4 (utils::getId(), Linestrings[4].invert(), Linestrings[7].invert());
    Lanelet lane5 (utils::getId(), Linestrings[5].invert(), Linestrings[8].invert());

    Lanelet lane6 (utils::getId(), Linestrings[10].invert(), Linestrings[9].invert());
    Lanelet lane7 (utils::getId(), Linestrings[10], Linestrings[11]);

    Lanelet lane8 (utils::getId(), Linestrings[14], Linestrings[12]);
    Lanelet lane9 (utils::getId(), Linestrings[15].invert(), Linestrings[13].invert());
    Lanelet lane10 (utils::getId(), Linestrings[14].invert(), Linestrings[16].invert());
    Lanelet lane11 (utils::getId(), Linestrings[15], Linestrings[17]);

    Lanes.push_back(lane0); Lanes.push_back(lane1); Lanes.push_back(lane2); Lanes.push_back(lane3); Lanes.push_back(lane4); Lanes.push_back(lane5); Lanes.push_back(lane6);
    Lanes.push_back(lane7); Lanes.push_back(lane8); Lanes.push_back(lane9); Lanes.push_back(lane10); Lanes.push_back(lane11);

    for (int l = 0; l < Lanes.size(); l++){
      //Lanes[l].attributes()[AttributeName::Subtype] = AttributeValueString::Road;
    }

    map = utils::createMap(Lanes);
    lanelet::Origin origin({0,0});
    write(tempfile("map.osm"), *map, origin);
  }