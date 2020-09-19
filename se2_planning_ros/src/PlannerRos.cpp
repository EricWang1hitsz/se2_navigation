/*
 * PlannerRos.cpp
 *
 *  Created on: Apr 5, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/PlannerRos.hpp"

namespace se2_planning {

PlannerRos::PlannerRos(ros::NodeHandlePtr nh) : nh_(nh) {}

void PlannerRos::publishPath() const {
  throw std::runtime_error("Publish path not implemented");
}

void PlannerRos::setPlanningStrategy(std::shared_ptr<Planner> planner) {
  planner_ = planner;
}

void PlannerRos::setStartingState(const State& startingState) {
  planner_->setStartingState(startingState);
}
void PlannerRos::setGoalState(const State& goalState) {
  planner_->setGoalState(goalState);
}
bool PlannerRos::plan() {
  return planner_->plan();
}
void PlannerRos::getPath(Path* path) const {
  planner_->getPath(path);
}
// Strive4G8ness: get ompl path , not rs path.
ompl::geometric::PathGeometric PlannerRos::getOmplPath() const{
    return planner_->getOmplPath();
}
void PlannerRos::getOmplPath(ompl::geometric::PathGeometric *omplPath) const{
    planner_->getOmplPath(omplPath);
}
void PlannerRos::getOmplInterpolatedPath(ompl::geometric::PathGeometric *omplPath, double spatialResolution) const{
    planner_->getOmplInterpolatedPath(omplPath, spatialResolution);
}
bool PlannerRos::reset() {
  return planner_->reset();
}
bool PlannerRos::initialize() {
  return planner_->initialize();
}
void PlannerRos::getStartingState(State* startingState) {
  planner_->getStartingState(startingState);
}
void PlannerRos::getGoalState(State* goalState) {
  planner_->getGoalState(goalState);
}

}  // namespace se2_planning
