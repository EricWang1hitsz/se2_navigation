/*
 * PlannerRos.hpp
 *
 *  Created on: Apr 5, 2020
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include "se2_navigation_msgs/RequestPathSrv.h"
#include "se2_planning/Planner.hpp"

namespace se2_planning {

class PlannerRos : public Planner {
 public:
  void setStartingState(const State& startingState) override;
  void setGoalState(const State& goalState) override;
  bool plan() override;
  void getPath(Path* path) const override;
  //!Eric_Wang:
  ompl::geometric::PathGeometric getOmplPath() const override;
  void getOmplPath(ompl::geometric::PathGeometric* omplPath) const override;
  void getOmplInterpolatedPath(ompl::geometric::PathGeometric* omplPath, double spatialResolution) const override;
  bool reset() override;
  bool initialize() override;
  void getStartingState(State* startingState) override;
  void getGoalState(State* goalState) override;

  void setPlanningStrategy(std::shared_ptr<Planner> planner);
  virtual void publishPath() const;

 protected:
  using PlanningService = se2_navigation_msgs::RequestPathSrv;

  explicit PlannerRos(ros::NodeHandlePtr nh);
  ~PlannerRos() override = default;
  virtual bool planningService(PlanningService::Request& req, PlanningService::Response& res) = 0;

  ros::NodeHandlePtr nh_;
  std::shared_ptr<Planner> planner_;
};

} /* namespace se2_planning*/
