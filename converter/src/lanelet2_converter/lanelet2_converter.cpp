#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace lanelet;
using namespace std;
using Eigen::Vector3d;

double max_angle = M_PI / 6;
lanelet::LaneletMapPtr turn_directions(lanelet::LaneletMapPtr map);

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lanelet2_converter");

  string pathfile;
  string path;
  string filename;

  cout << "Enter a path to a Lanelet2 map:" << endl;
  cin >> pathfile;
  cout << endl;
  cout << "Enter a new filename (without the file extention): " << endl;
  cin >> filename;
  cout << endl;
  /*pathfile = "/home/gel3abt/Maps/lanelet2_example_ejqc0B/lanelet2_map.osm";
  filename = "output";*/

  path = pathfile;

  int pos = pathfile.rfind("/");
  int pop = path.size() - (pos + 1);
  for (int i = 0; i < pop; i++)
  {
    path.pop_back();
  }

  // cout << pathfile << endl;
  // cout << path << endl;

  lanelet::Origin origin({0, 0});
  projection::UtmProjector projector(origin);
  LaneletMapPtr map = load(pathfile, projector);

  // Add 'ele' attribute to every point
  for (auto &point : map->pointLayer)
  {
    //point.attributes()["ele"] = "0";
  }

  // Add the Subtype road to every lanelet
  for (auto &Lanes : map->laneletLayer)
  {
    Lanes.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
  }

  // find following lanes and add turn directions if there are more than one follower
  map = turn_directions(map);

  string output_filename = path + filename + ".osm";

  write(output_filename, *map, origin);

  rclcpp::shutdown();
  return 0;
}

/**
  @brief Add turn directions for overlaying lanelets
  @details Find lanelets with muliple following lanelets and calculate their angle to the lanelet to add turn directions
*/
lanelet::LaneletMapPtr turn_directions(lanelet::LaneletMapPtr map)
{
  traffic_rules::TrafficRulesPtr trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

  LaneletMapConstPtr debugLaneletMap = routingGraph->getDebugLaneletMap(lanelet::routing::RoutingCostId(0));
  write(std::string("routing_graph.osm"), *debugLaneletMap);


  for (const auto &lanelet : map->laneletLayer)
  {
     //cout << routingGraph->following(lanelet).size() << endl;
    if (routingGraph->following(lanelet).size() > 1)
    {
      //cout << routingGraph->following(lanelet).size() << endl;
      lanelet::ConstLanelets following_lanes = routingGraph->following(lanelet);
      vector<double> angles;

      for (int k = 0; k < following_lanes.size(); k++)
      {
        ConstLineString3d centerline = lanelet.centerline();
        ConstLineString3d center_following = following_lanes[k].centerline();
        ConstPoint3d A1 = centerline.back();
        ConstPoint3d A2 = centerline[centerline.size() - 2];
        ConstPoint3d P = center_following.back();

        Vector3d p_line1(A1.x(), A1.y(), A1.z());
        Vector3d p_line2(A2.x(), A2.y(), A2.z());
        Vector3d point(P.x(), P.y(), P.z());

        // d = abs((p - q) x u) / abs(u)
        Vector3d dir_line = p_line1 - p_line2;
        Vector3d b_m_p_line = point - p_line1;
        Vector3d dir_x_b_m_p_line = b_m_p_line.cross(dir_line);
        double dist_point_line = dir_x_b_m_p_line.norm() / dir_line.norm();

        Vector3d p_line1_point = point - p_line1;
        double dist_p_line1_point = p_line1_point.norm();

        double angle = asin(dist_point_line / dist_p_line1_point);
        // cout << "Gegen " << dist << endl;
        // cout << "Hypo " << dist_p_line1_point << endl;

        Vector3d cross_prod = dir_line.cross(p_line1_point);
        if (cross_prod.z() < 0)
        {
          angle = -angle;
        }

        // cout << "Winkel " << following_lanes[k].id() << " : " << angle << endl;
        angles.push_back(angle);
      }

      int pos;
      double smallest = 1000000;
      for (int m = 0; m < angles.size(); m++)
      {
        if (abs(angles[m]) < smallest)
        {
          smallest = abs(angles[m]);
          pos = m;
        }
      }

      for (int n = 0; n < following_lanes.size(); n++)
      {
        if (n == pos && abs(angles[n]) < max_angle)
        {
          map->laneletLayer.get(following_lanes[n].id()).attributes()["turn_direction"] = "straight";
        }
        else if (angles[n] > 0)
        {
          map->laneletLayer.get(following_lanes[n].id()).attributes()["turn_direction"] = "left";
        }
        else if (angles[n] < 0)
        {
          map->laneletLayer.get(following_lanes[n].id()).attributes()["turn_direction"] = "right";
        }
      }
    }
  }
  return map;
}