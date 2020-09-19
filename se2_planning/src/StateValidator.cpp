/*
 * StateValidator.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#include "se2_planning/StateValidator.hpp"
#include <ros/ros.h>

using namespace grid_map;
namespace se2_planning {

void StateValidator::initialize() {}

bool StateValidator::isInitialized() const {
  return true;
}

SE2stateValidator::SE2stateValidator()
{
    //loadGlobalMapService_ = nh_.advertiseService("load_global_map", &SE2stateValidator::loadGlobalTraversabilityMap, this);
    //globalTraversabilityMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/fused_traversability_map", 100);
    fusedTraversabilityMapSubcriber_ = nh_.subscribe("/fused_traversability_map", 100, &SE2stateValidator::fusedMapCallback, this);
    ROS_INFO(" State validator initialized. ");
}

SE2stateValidator::~SE2stateValidator()
{
    ROS_INFO(" deconstructed ");
}

bool SE2stateValidator::isStateValid(const State& state) const {
  // implement something useful here
  ROS_INFO_ONCE("State!!!");
  const SE2state *se2state = state.as<SE2state>();
  double x = se2state->x_;
  double y = se2state->y_;
  double yaw = se2state->yaw_;
  double radius = 0.45;
  Eigen::Vector2d center(x, y);
  grid_map::Position position_(x, y);
  double traversability = 0;
  int nCells = 0;
  for(grid_map::SpiralIterator iterator(traversability_map_update_, center, radius); !iterator.isPastEnd(); ++iterator)
  {
      float currentPositionIsTraversale = traversability_map_update_.at("traversability", *iterator);
      if(!traversability_map_update_.isValid(*iterator, "traversability"))
          ROS_WARN("Invalid data in traversability map");
      if(currentPositionIsTraversale > 0.50){
          nCells++;
          traversability += traversability_map_update_.at("traversability", *iterator);
      }
      else{
          traversability = 0;
          return false;
      }
  }
  traversability /= nCells;
  //ROS_WARN_STREAM("traversability is: " << traversability);
  if(traversability > 0.7)
      return true;
  else
      return false;
}

bool SE2stateValidator::loadGlobalTraversabilityMap(se2_navigation_msgs::RequestNavigationMapSrv::Request &request,
                                                    se2_navigation_msgs::RequestNavigationMapSrv::Response &response)
{
    ROS_INFO("State validator checker.");
    std::string topic = request.topic_name;
    std::string path_file = request.file_path;
    GridMap gridMap;
    grid_map_msgs::GridMap message_;
    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);
    traversability_map_update_ = gridMap;
    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));
    GridMapRosConverter::toMessage(gridMap, message_);
    globalTraversabilityMapPublisher_.publish(message_);
    ROS_WARN("Load global traversability map success.");
    return true;
}

void SE2stateValidator::fusedMapCallback(grid_map_msgs::GridMap message)
{
    // receive updated map once when load global map and map fusion.
    ROS_INFO("Receive update map Once. ");
    GridMap gridMap;
    GridMapRosConverter::fromMessage(message, gridMap);
    traversability_map_update_ = gridMap;
}
}  // namespace se2_planning
