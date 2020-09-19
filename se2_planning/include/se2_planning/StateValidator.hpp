/*
 * StateValidator.hpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#pragma once
#include "se2_planning/State.hpp"
#include "se2_navigation_msgs/RequestNavigationMapSrv.h"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/ros.h>

using namespace grid_map;
using namespace ompl::base;
namespace se2_planning {

class StateValidator {
 public:
  StateValidator() = default;
  virtual ~StateValidator() = default;
  virtual bool isStateValid(const State& state) const = 0;
  virtual void initialize();
  virtual bool isInitialized() const;
};

class SE2stateValidator : public StateValidator {
 public:
//  SE2stateValidator() = default;
//  ~SE2stateValidator() override = default;
  SE2stateValidator();
  ~SE2stateValidator();
  bool isStateValid(const State& state) const override;
  bool loadGlobalTraversabilityMap(se2_navigation_msgs::RequestNavigationMapSrv::Request& request,
                                   se2_navigation_msgs::RequestNavigationMapSrv::Response& response);
  void fusedMapCallback(grid_map_msgs::GridMap message);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer loadGlobalMapService_;
  ros::Publisher globalTraversabilityMapPublisher_;
  ros::Subscriber fusedTraversabilityMapSubcriber_;

  GridMap traversability_map_update_;
};

} /* namespace se2_planning */
