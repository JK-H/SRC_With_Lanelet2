/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mission_planner/lanelet2_impl/mission_planner_lanelet2.h>
#include <mission_planner/lanelet2_impl/route_handler.h>
#include <mission_planner/lanelet2_impl/utility_functions.h>
#include <mission_planner/lanelet2_impl/TSP_concorde.h>
#include <mission_planner/lanelet2_impl/TSP_LKH.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <unordered_set>
#include <stack>
#include <time.h>

namespace
{
RouteSections combineConsecutiveRouteSections(
  const RouteSections & route_sections1, const RouteSections & route_sections2)
{
  RouteSections route_sections;
  route_sections.reserve(route_sections1.size() + route_sections2.size());
  if (!route_sections1.empty()) {
    // remove end route section because it is overlapping with first one in next route_section
    route_sections.insert(route_sections.end(), route_sections1.begin(), route_sections1.end() - 1);
  }
  if (!route_sections2.empty()) {
    route_sections.insert(route_sections.end(), route_sections2.begin(), route_sections2.end());
  }
  return route_sections;
}

bool isRouteLooped(const RouteSections & route_sections)
{
  for (std::size_t i = 0; i < route_sections.size(); i++) {
    const auto & route_section = route_sections.at(i);
    for (const auto & lane_id : route_section.lane_ids) {
      for (std::size_t j = i + 1; j < route_sections.size(); j++) {
        const auto & future_route_section = route_sections.at(j);
        if (exists(future_route_section.lane_ids, lane_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

constexpr double normalizeRadian(
  const double rad, const double min_rad = -M_PI, const double max_rad = M_PI)
{
  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad < value && value <= max_rad)
    return value;
  else
    return value - std::copysign(2 * M_PI, value);
}

bool isInLane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

bool isInParkingSpace(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_space : parking_spaces) {
    lanelet::ConstPolygon3d parking_space_polygon;
    if (!lanelet::utils::lineStringWithWidthToPolygon(parking_space, &parking_space_polygon)) {
      continue;
    }

    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_space_polygon).basicPolygon(),
      lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

bool isInParkingLot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_lot : parking_lots) {
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_lot).basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

}  // anonymous namespace

namespace mission_planner
{
MissionPlannerLanelet2::MissionPlannerLanelet2() : is_graph_ready_(false)
{
  map_subscriber_ =
    pnh_.subscribe("input/vector_map", 10, &MissionPlannerLanelet2::mapCallback, this);
}

void MissionPlannerLanelet2::mapCallback(const autoware_lanelet2_msgs::MapBin & msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  is_graph_ready_ = true;
  initializeNode2laneletHash();
}

bool MissionPlannerLanelet2::isRoutingGraphReady() const { return (is_graph_ready_); }

void MissionPlannerLanelet2::visualizeRoute(const autoware_planning_msgs::Route & route) const
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets normal_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  for (const auto & route_section : route.route_sections) {
    for (const auto & lane_id : route_section.lane_ids) {
      auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_lane_id == lane_id) {
        goal_lanelets.push_back(lanelet);
      } else if (exists(route_section.continued_lane_ids, lane_id)) {
        normal_lanelets.push_back(lanelet);
      } else {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  std_msgs::ColorRGBA cl_route, cl_ll_borders, cl_end, cl_normal, cl_goal;
  setColor(&cl_route, 0.0, 0.7, 0.2, 0.5);
  setColor(&cl_goal, 0.0, 0.0, 0.0, 0.0);
  setColor(&cl_end, 0.0, 0.0, 0.0, 0.0);
  setColor(&cl_normal, 0.0, 0.0, 0.0, 0.0);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);

  visualization_msgs::MarkerArray route_marker_array;
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "normal_lanelets", normal_lanelets, cl_normal));
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));
  marker_publisher_.publish(route_marker_array);
}

bool MissionPlannerLanelet2::isGoalValid() const
{
  lanelet::Lanelet closest_lanelet;
  if (!getClosestLanelet(goal_pose_.pose, lanelet_map_ptr_, &closest_lanelet)) {
    return false;
  }
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal_pose_.pose.position);

  if (isInLane(closest_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_lanelet, goal_pose_.pose.position);
    const auto goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);

    constexpr double th_angle = M_PI / 4;

    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  // check if goal is in parking space
  const auto parking_spaces = lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  if (isInParkingSpace(parking_spaces, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in parking lot
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);
  if (isInParkingLot(parking_lots, goal_lanelet_pt)) {
    return true;
  }

  return false;
}

autoware_planning_msgs::Route MissionPlannerLanelet2::planRoute()
{
  std::stringstream ss;
  for (const auto & checkpoint : checkpoints_) {
    ss << "x: " << checkpoint.pose.position.x << " "
       << "y: " << checkpoint.pose.position.y << std::endl;
  }
  ROS_INFO_STREAM("start planning route with checkpoints: " << std::endl << ss.str());

  autoware_planning_msgs::Route route_msg;
  RouteSections route_sections;

  if (!isGoalValid()) {
    ROS_WARN("Goal is not valid! Please check position and angle of goal_pose");
    return route_msg;
  }

  for (std::size_t i = 1; i < checkpoints_.size(); i++) {
    const auto start_checkpoint = checkpoints_.at(i - 1);
    const auto goal_checkpoint = checkpoints_.at(i);
    lanelet::ConstLanelets path_lanelets;
    if (!planFullCoveragePathByTSP(start_checkpoint, goal_checkpoint, &path_lanelets)) {
      return route_msg;
    }

    ROS_INFO_STREAM("planFullCoveragePath is compeleted!" << std::endl);

    RouteHandler route_handler(lanelet_map_ptr_, routing_graph_ptr_, path_lanelets);
    // const auto main_lanelets = getMainLanelets(path_lanelets, route_handler);

    // //  create routesections
    const auto local_route_sections = createRouteSections(path_lanelets, route_handler);
    route_sections = combineConsecutiveRouteSections(route_sections, local_route_sections);
  }

  if (isRouteLooped(route_sections)) {
    ROS_WARN("Loop detected within route! Be aware that looped route is not debugged!");
  }

  route_msg.header.stamp = ros::Time::now();
  route_msg.header.frame_id = map_frame_;
  route_msg.route_sections = route_sections;
  route_msg.goal_pose = goal_pose_.pose;

  return route_msg;
}

bool MissionPlannerLanelet2::planPathBetweenCheckpoints(
  const geometry_msgs::PoseStamped & start_checkpoint,
  const geometry_msgs::PoseStamped & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets_ptr) const
{
  lanelet::Lanelet start_lanelet;
  if (!getClosestLanelet(start_checkpoint.pose, lanelet_map_ptr_, &start_lanelet)) {
    return false;
  }
  lanelet::Lanelet goal_lanelet;
  if (!getClosestLanelet(goal_checkpoint.pose, lanelet_map_ptr_, &goal_lanelet)) {
    return false;
  }

  // get all possible lanes that can be used to reach goal (including all possible lane change)
  lanelet::Optional<lanelet::routing::Route> optional_route =
    routing_graph_ptr_->getRoute(start_lanelet, goal_lanelet, 0);
  if (!optional_route) {
    ROS_ERROR_STREAM(
      "Failed to find a proper path!"
      << std::endl
      << "start checkpoint: " << toString(start_pose_.pose) << std::endl
      << "goal checkpoint: " << toString(goal_pose_.pose) << std::endl
      << "start lane id: " << start_lanelet.id() << std::endl
      << "goal lane id: " << goal_lanelet.id() << std::endl);
    return false;
  }

  const auto shortest_path = optional_route->shortestPath();
  for (const auto & llt : shortest_path) {
    path_lanelets_ptr->push_back(llt);
  }
  return true;
}

lanelet::ConstLanelets MissionPlannerLanelet2::getMainLanelets(
  const lanelet::ConstLanelets & path_lanelets, const RouteHandler & route_handler)
{
  auto lanelet_sequence = route_handler.getLaneletSequence(path_lanelets.back());
  lanelet::ConstLanelets main_lanelets;
  while (!lanelet_sequence.empty()) {
    main_lanelets.insert(main_lanelets.begin(), lanelet_sequence.begin(), lanelet_sequence.end());
    lanelet_sequence = route_handler.getPreviousLaneletSequence(lanelet_sequence);
  }
  return main_lanelets;
}

RouteSections MissionPlannerLanelet2::createRouteSections(
  const lanelet::ConstLanelets & main_path, const RouteHandler & route_handler)
{
  RouteSections route_sections;

  if (main_path.empty()) return route_sections;

  for (const auto & main_llt : main_path) {
    autoware_planning_msgs::RouteSection route_section_msg;
    lanelet::ConstLanelets route_section_lanelets = route_handler.getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_lane_id = main_llt.id();
    for (const auto & section_llt : route_section_lanelets) {
      route_section_msg.lane_ids.push_back(section_llt.id());
      lanelet::ConstLanelet next_lanelet;
      if (route_handler.getNextLaneletWithinRoute(section_llt, &next_lanelet)) {
        route_section_msg.continued_lane_ids.push_back(section_llt.id());
      }
    }
    route_sections.push_back(route_section_msg);
  }
  return route_sections;
}

bool MissionPlannerLanelet2::expandPathToTheLanelet(
    lanelet::ConstLanelets * path_lanelets_ptr,
    const lanelet::ConstLanelet & goal_lanelet) const
{
  if(path_lanelets_ptr->empty()){
    ROS_ERROR_STREAM("path_lanelets_ptr is empty!");
    return false;
  }
  lanelet::Optional<lanelet::routing::LaneletPath> go_path =
      routing_graph_ptr_->shortestPath(path_lanelets_ptr->back(), goal_lanelet, 0, false);
  if(!go_path){
    ROS_ERROR_STREAM(
      "Failed to find a proper go_path!"
      << std::endl
      << "start lane id: " << path_lanelets_ptr->back().id() << std::endl
      << "goal lane id: " << goal_lanelet.id() << std::endl);
    return false;
  }
  path_lanelets_ptr->pop_back();
  for (const auto & llt : *go_path) {
    path_lanelets_ptr->push_back(llt);
  }
  return true;
}

// full coverage 
bool MissionPlannerLanelet2::planFullCoveragePath(
  const geometry_msgs::PoseStamped & start_checkpoint,
  const geometry_msgs::PoseStamped & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets_ptr) const
{
  clock_t begin = clock();

  lanelet::Lanelet start_lanelet;
  if (!getClosestLanelet(start_checkpoint.pose, lanelet_map_ptr_, &start_lanelet)) {
    return false;
  }
  lanelet::Lanelet goal_lanelet;
  if (!getClosestLanelet(goal_checkpoint.pose, lanelet_map_ptr_, &goal_lanelet)) {
    return false;
  }

  // lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get();
  lanelet::ConstLanelets full_coverage_path;
  std::unordered_set<lanelet::Id> visited_lanelet;
  std::stack<lanelet::ConstLanelet> dfs_stk;
  // visited_lanelet.emplace(start_lanelet.id());
  dfs_stk.push(start_lanelet);
  full_coverage_path.push_back(start_lanelet);
  while (!dfs_stk.empty())
  // while(full_coverage_path.size() < 30)
  {
    lanelet::ConstLanelet pose_lanelet = full_coverage_path.back();    // the pose of vehicle now
    lanelet::ConstLanelet cur_lanelet = dfs_stk.top(); // the lanelet that will be processed 
    dfs_stk.pop();
    
    if(!visited_lanelet.count(cur_lanelet.id())){
      // connect backtracking lanelet with current pose lanelet
      if (*(routing_graph_ptr_->routingRelation(pose_lanelet, cur_lanelet)) != lanelet::routing::RelationType::Successor) {
        lanelet::Optional<lanelet::routing::Route> back_route =
          routing_graph_ptr_->getRoute(pose_lanelet, cur_lanelet, 0);
        if (!back_route) {
          ROS_ERROR_STREAM(
            "Failed to find a proper back_route!"
            << std::endl
            << "start lane id: " << pose_lanelet.id() << std::endl
            << "goal lane id: " << cur_lanelet.id() << std::endl);
          return false;
        }
        const lanelet::routing::LaneletPath back_path = back_route->shortestPath();
        full_coverage_path.pop_back();
        for (const auto& llt : back_path) {
          if(!visited_lanelet.count(llt.id())){
            visited_lanelet.emplace(llt.id());
          }
          full_coverage_path.push_back(llt);
          lanelet::ConstLanelets following_llts = routing_graph_ptr_->following(llt);
          for (const auto & following_llt : following_llts){
            dfs_stk.push(following_llt);
          }
        }
      }else{
        lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(cur_lanelet);
        for (const auto & following_llt : following_lanelets){
          dfs_stk.push(following_llt);
        }
        
        visited_lanelet.emplace(cur_lanelet.id());
        full_coverage_path.push_back(cur_lanelet);
      }
    }
  }
  // go to the goal lanelet
  lanelet::Optional<lanelet::routing::Route> last_route =
            routing_graph_ptr_->getRoute(full_coverage_path.back(), goal_lanelet, 0);
  if (!last_route) {
    ROS_ERROR_STREAM(
      "Failed to find a proper back_route!"
      << std::endl
      << "start lane id: " << full_coverage_path.back().id() << std::endl
      << "goal lane id: " << goal_lanelet.id() << std::endl);
    return false;
  }
  const lanelet::routing::LaneletPath last_path = last_route->shortestPath();
  full_coverage_path.pop_back();
  for (const auto& llt : last_path) {
    if(!visited_lanelet.count(llt.id())){
      visited_lanelet.emplace(llt.id());
    }
    full_coverage_path.push_back(llt);
  }

  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  std::cout << "elapsed time: " << elapsed_secs << "\n";

  ROS_INFO_STREAM("laneletLayer size: " << lanelet_map_ptr_->laneletLayer.size());
  ROS_INFO_STREAM("passableSubmap() size: " << routing_graph_ptr_->passableSubmap()->laneletLayer.size());
  ROS_INFO_STREAM("visited_lanelet_set size: " << visited_lanelet.size());
  ROS_INFO_STREAM("full_coverage_path size: " << full_coverage_path.size() << std::endl);
  for (auto it = full_coverage_path.begin(); it != full_coverage_path.end(); it++) {
    lanelet::ConstLanelet curllt = *it;
    path_lanelets_ptr->push_back(curllt);
    ROS_INFO_STREAM("path_lanelets_ptr id [" << path_lanelets_ptr->size() << "] : " << curllt.id());
  }

  return true;
}

// get weight element of matrix
double MissionPlannerLanelet2::getWeightOfAjacentMatrix(
  const lanelet::ConstLanelet& from,
  const lanelet::ConstLanelet& to) const
{
  double total_weight = 0;
  lanelet::ConstLanelets weight_path{from};
  if(!expandPathToTheLanelet(&weight_path, to)) {
    return 0;
  }

  for (auto it = weight_path.begin(); it != weight_path.end() - 1; it++) {
    lanelet::Optional<double> edge_cost = routing_graph_ptr_->getEdgeCost(*it, *(it + 1));
    if(!edge_cost){
      ROS_ERROR_STREAM(
      "Failed to get the cost of edge!"
      << std::endl
      << "from lane id: " << it->id() << std::endl
      << "to lane id: " << (it + 1)->id() << std::endl);
      break;
    }
    total_weight += *edge_cost;
  }
  return total_weight;
}

void MissionPlannerLanelet2::initializeNode2laneletHash()
{
  auto umap_it = routing_graph_ptr_->passableSubmap()->laneletLayer.begin();
  auto umap_end = routing_graph_ptr_->passableSubmap()->laneletLayer.end();
  for (int node_index = 0; umap_it != umap_end; node_index++, umap_it++){
      node2lanelet_hash_.emplace(node_index, umap_it->id());
  }
}

// get weight matrix
std::vector<std::vector<double>> MissionPlannerLanelet2::getAjacentMatrix() const
{
  if(routing_graph_ptr_->passableSubmap()->laneletLayer.size() != node2lanelet_hash_.size()){
    ROS_ERROR_STREAM("node2lanelet_hash_ is not correct!");
    return std::vector<std::vector<double>>{};
  }
  int dimension = node2lanelet_hash_.size();
  std::vector<std::vector<double>> aj_matrix(dimension + 1, std::vector<double>(dimension + 1, 0));
  for (int row = 0; row < dimension; row++){
      for (int col = 0; col < dimension; col++){
        if(row != col){
          lanelet::ConstLanelet fromllt = lanelet_map_ptr_->laneletLayer.get(node2lanelet_hash_.at(row));
          lanelet::ConstLanelet tollt = lanelet_map_ptr_->laneletLayer.get(node2lanelet_hash_.at(col));
          aj_matrix[row][col] = getWeightOfAjacentMatrix(fromllt, tollt);
        }
        else {
          aj_matrix[row][col] = 1000000.0;
        }
      }
  }
  aj_matrix[dimension][dimension] = 1000000.0;
  ROS_INFO_STREAM("Distance matrix is created!");
  return aj_matrix;
}

// plan full coverage path by TSP solver
bool MissionPlannerLanelet2::planFullCoveragePathByTSP(
  const geometry_msgs::PoseStamped & start_checkpoint,
  const geometry_msgs::PoseStamped & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets_ptr) const
{
  clock_t begin = clock();

  lanelet::Lanelet start_lanelet;
  if (!getClosestLanelet(start_checkpoint.pose, lanelet_map_ptr_, &start_lanelet)) {
    return false;
  }
  lanelet::Lanelet goal_lanelet;
  if (!getClosestLanelet(goal_checkpoint.pose, lanelet_map_ptr_, &goal_lanelet)) {
    return false;
  }

  std::vector<std::vector<double>> ajacent_matrix = getAjacentMatrix();
  std::vector<int> optimal_node_order;      // Hamilton cycle

  // ConcordeTSPSolver tsp_solver;
  // optimal_node_order = tsp_solver.solveConcordeTSP(ajacent_matrix);
  LKHTSPSolver tsp_solver;
  optimal_node_order = tsp_solver.solveLKHTSP(ajacent_matrix);

// 
  double verify_solution = 0;
  for (auto it = optimal_node_order.begin(); it != optimal_node_order.end() - 1; it++) {
      verify_solution += ajacent_matrix[*it][*(it + 1)];
  }
  verify_solution += ajacent_matrix[optimal_node_order.back()][0];
  ROS_INFO_STREAM("verify_solution : " << verify_solution);
  //

  // convert to lanelet order
  std::vector<lanelet::Id> lanelet_id_order;
  for (int& index: optimal_node_order) {
    // remove the added node at last
    if(index < optimal_node_order.size() - 1)
      lanelet_id_order.push_back(node2lanelet_hash_.at(index));
  }

  // open the Hamilton cycle with the start_lanelet at the beginning
  lanelet::Id start_id = start_lanelet.id();
  unsigned int start_id_position;
  // find position of the start id in the order
	for (unsigned int i = 0; i < lanelet_id_order.size(); i++) {
		if (lanelet_id_order[i] == start_id){
			start_id_position = i;
		}
	}
  // sort the vector starting at start_id
  std::vector<lanelet::Id> final_lanelet_id_order;
  for (unsigned int i = start_id_position; i < lanelet_id_order.size(); i++) {
		final_lanelet_id_order.push_back(lanelet_id_order[i]);
	}
	for (unsigned int i = 0; i < start_id_position; i++) {
		final_lanelet_id_order.push_back(lanelet_id_order[i]);
	}

  lanelet::ConstLanelets full_coverage_path{start_lanelet};
  final_lanelet_id_order.erase(final_lanelet_id_order.begin());
  for (const lanelet::Id& llt_id : final_lanelet_id_order) {
    if(!expandPathToTheLanelet(&full_coverage_path, lanelet_map_ptr_->laneletLayer.get(llt_id)))
      return false;
  }

  ROS_INFO_STREAM("Cost of full_coverage_path after TSP solved: " << getPathCost(full_coverage_path));

  // go to the goal_lanelet
  if(!expandPathToTheLanelet(&full_coverage_path, goal_lanelet))
    return false;

  ROS_INFO_STREAM("Cost of full_coverage_path finally: " << getPathCost(full_coverage_path));

  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  ROS_INFO_STREAM("elapsed time: " << elapsed_secs);

  ROS_INFO_STREAM("laneletLayer size: " << lanelet_map_ptr_->laneletLayer.size());
  ROS_INFO_STREAM("passableSubmap() size: " << routing_graph_ptr_->passableSubmap()->laneletLayer.size());
  ROS_INFO_STREAM("full_coverage_path size: " << full_coverage_path.size() << std::endl);
  for (auto it = full_coverage_path.begin(); it != full_coverage_path.end(); it++) {
    lanelet::ConstLanelet curllt = *it;
    path_lanelets_ptr->push_back(curllt);
    ROS_INFO_STREAM("path_lanelets_ptr id [" << path_lanelets_ptr->size() << "] : " << curllt.id());
  }

  return true;
}

// get cost of a path
double MissionPlannerLanelet2::getPathCost(const lanelet::ConstLanelets& path) const
{
  double total_cost = 0;
  for (auto it = path.begin(); it != path.end() - 1; it++) {
    lanelet::Optional<lanelet::routing::RelationType> relation = 
      routing_graph_ptr_->routingRelation(*it, *(it + 1));
    if (!!relation && (*relation == lanelet::routing::RelationType::Successor
        || *relation == lanelet::routing::RelationType::Left
        || *relation == lanelet::routing::RelationType::Right)){
      lanelet::Optional<double> edge_cost = routing_graph_ptr_->getEdgeCost(*it, *(it + 1));
      if (!edge_cost) {
        ROS_ERROR_STREAM(
            "Failed to get the cost of edge!"
            << std::endl
            << "from lane id: " << it->id() << std::endl
            << "to lane id: " << (it + 1)->id() << std::endl);
        break;
      }
      total_cost += *edge_cost;
    }
    else{
      ROS_ERROR_STREAM("somewhere is unreachable in path!" << std::endl
                        << "from lane id: " << it->id() << std::endl
                        << "to lane id: " << (it + 1)->id() << std::endl);
      break;
    }
  }

  return total_cost;
}

void MissionPlannerLanelet2::visualizeRouteStepByStep(const autoware_planning_msgs::Route & route) const
{
  lanelet::ConstLanelets route_lanelets;
  ros::Rate r(4);

  for (const auto & route_section : route.route_sections) {
    auto lanelet = lanelet_map_ptr_->laneletLayer.get(route_section.preferred_lane_id);
    route_lanelets.push_back(lanelet);

    lanelet::ConstLanelets current_lanelets;
    current_lanelets.push_back(lanelet);

    std_msgs::ColorRGBA cl_route, cl_cur_llt;
    setColor(&cl_route, 1.0, 1.0, 0.0, 0.5);
    setColor(&cl_cur_llt, 1.0, 0.0, 0.0, 0.5);
    
    visualization_msgs::MarkerArray route_marker_array;
    insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
    insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "current_lanelets", current_lanelets, cl_cur_llt));

    marker_publisher_.publish(route_marker_array);
    r.sleep();
    // marker.lifetime = ros::Duration();
  }

}


}  // namespace mission_planner
