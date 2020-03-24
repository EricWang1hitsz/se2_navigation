/*
 * PathTracker.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/path_tracking/PathTracker.hpp"

#include "pure_pursuit_core/heading_control/HeadingController.hpp"
#include "pure_pursuit_core/math.hpp"
#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"
#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

void PathTracker::setHeadingController(std::shared_ptr<HeadingController> ctrl) {
  headingController_ = ctrl;
}
void PathTracker::setVelocityController(std::shared_ptr<LongitudinalVelocityController> ctrl) {
  velocityController_ = ctrl;
}
void PathTracker::setProgressValidator(std::shared_ptr<ProgressValidator> validator) {
  progressValidator_ = validator;
}

void PathTracker::setPathPreprocessor(std::shared_ptr<PathPreprocessor> pathPreprocessor) {
  pathPreprocessor_ = pathPreprocessor;
}

double PathTracker::getTurningRadius() const {
  return turningRadius_;
}
double PathTracker::getYawRate() const {
  return yawRate_;
}
double PathTracker::getSteeringAngle() const {
  return steeringAngle_;
}

double PathTracker::getLongitudinalVelocity() const {
  return longitudinalVelocity_;
}

void PathTracker::importCurrentPath(const Path& path) {
  if (path.segment_.empty()) {
    throw std::runtime_error("empty path");
  }
  currentPath_ = path;
}

bool PathTracker::initialize() {
  return true;
}
bool PathTracker::advance() {
  advanceStateMachine();
  advanceControllers();
  return true;
}

void PathTracker::updateRobotState(const RobotState& robotState) {
  currentRobotState_ = robotState;
}

} /* namespace pure_pursuit */
