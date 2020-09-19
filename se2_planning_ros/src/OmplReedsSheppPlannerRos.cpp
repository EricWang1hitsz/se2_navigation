/*
 * OmplReedsSheppPlannerRos.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ob = ompl::base;

namespace se2_planning {

OmplReedsSheppPlannerRos::OmplReedsSheppPlannerRos(ros::NodeHandlePtr nh) : BASE(nh) {
  initRos();
}

void OmplReedsSheppPlannerRos::setParameters(const OmplReedsSheppPlannerRosParameters& parameters) {
  parameters_ = parameters;
}

bool OmplReedsSheppPlannerRos::initialize() {
  bool result = BASE::initialize();
  return result;
}
bool OmplReedsSheppPlannerRos::plan() {
  bool result = BASE::plan();
  if (result) {
    planSeqNumber_++;
  }
  std::thread t([this]() {
    publishPath();
    publishPathNavMsgs();
    publishOmplPathNavMsgs();
  });
  t.detach();
  return result;
}

bool OmplReedsSheppPlannerRos::planningService(PlanningService::Request& req, PlanningService::Response& res) {
  const auto start = se2_planning::convert(req.pathRequest.startingPose);
  const auto goal = se2_planning::convert(req.pathRequest.goalPose);
  setStartingState(start);
  setGoalState(goal);
  bool result = plan();

  res.status = result;

  return true;
}

void OmplReedsSheppPlannerRos::initRos() {
  pathNavMsgsPublisher_ = nh_->advertise<nav_msgs::Path>(parameters_.pathNavMsgTopic_, 1, true);
  planningService_ = nh_->advertiseService(parameters_.planningSerivceName_, &OmplReedsSheppPlannerRos::planningService, this);
  pathPublisher_ = nh_->advertise<se2_navigation_msgs::PathMsg>(parameters_.pathMsgTopic_, 1);
  omplPathNavMsgsPublisher_ = nh_->advertise<nav_msgs::Path>("omplPathNavMsgs", 1, true);
}

void OmplReedsSheppPlannerRos::publishPathNavMsgs() const {
  ReedsSheppPath rsPath;
  planner_->as<OmplPlanner>()->getInterpolatedPath(&rsPath, parameters_.pathNavMsgResolution_);
  nav_msgs::Path msg = se2_planning::copyAllPoints(rsPath);
  msg.header.frame_id = parameters_.pathFrame_;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = planSeqNumber_;
  pathNavMsgsPublisher_.publish(msg);
  ROS_INFO_STREAM("Publishing ReedsShepp path nav msg, num states: " << msg.poses.size());
}

void OmplReedsSheppPlannerRos::publishPath() const {
  ReedsSheppPath rsPath;
  planner_->getPath(&rsPath); //reeds shepp path.
  se2_navigation_msgs::Path msg = se2_planning::convert(rsPath);
  msg.header_.frame_id = parameters_.pathFrame_;
  msg.header_.stamp = ros::Time::now();
  msg.header_.seq = planSeqNumber_;
  pathPublisher_.publish(se2_navigation_msgs::convert(msg));
  ROS_INFO_STREAM("Publishing ReedsShepp path, num states: " << rsPath.numPoints());
}

void OmplReedsSheppPlannerRos::publishOmplPathNavMsgs() const{
    auto Path = planner_->getOmplPath();
    ompl::geometric::PathGeometric *omplPath(&Path);
    planner_->getOmplInterpolatedPath(omplPath, parameters_.pathNavMsgResolution_);
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = parameters_.pathFrame_;
    msg.header.seq = planSeqNumber_;
    geometry_msgs::PoseStamped pose;

    for(std::size_t path_idx = 0; path_idx < omplPath->getStateCount(); path_idx++)
    {
        const ob::SE2StateSpace::StateType *se2state = omplPath->getState(path_idx)->as<ob::SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        pose.pose.position.x = pos->values[0];
        pose.pose.position.y = pos->values[1];
        pose.pose.position.z = 0.2;
        double yaw = pos->values[2];
        // Creat quaternion from yaw;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;
        msg.poses.push_back(pose);
        omplPathNavMsgsPublisher_.publish(msg);
    }
    ROS_INFO_STREAM("Publishing Ompl path nav msg, num states: " << msg.poses.size());
}

geometry_msgs::Pose convert(const ReedsSheppState& state, double z) {
  geometry_msgs::Pose pose;
  pose.position.x = state.x_;
  pose.position.y = state.y_;
  pose.position.z = z;
  pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw_);

  return pose;
}

ReedsSheppState convert(const geometry_msgs::Pose& state) {
  ReedsSheppState rsState;
  rsState.x_ = state.position.x;
  rsState.y_ = state.position.y;
  rsState.yaw_ = tf::getYaw(state.orientation);
  return rsState;
}

nav_msgs::Path copyAllPoints(const ReedsSheppPath& path) {
  nav_msgs::Path pathOut;
  pathOut.poses.reserve(path.numPoints());
  for (const auto& segment : path.segment_) {
    for (const auto& point : segment.point_) {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.pose = convert(point);
      pathOut.poses.push_back(poseStamped);
    }
  }
  return pathOut;
}

se2_navigation_msgs::Path convert(const ReedsSheppPath& path) {
  using DrivingDirection = se2_navigation_msgs::PathSegment::DrivingDirection;
  auto convertDirections = [](ReedsSheppPathSegment::Direction d) -> DrivingDirection {
    switch (d) {
      case ReedsSheppPathSegment::Direction::FWD:
        return DrivingDirection::Forward;
      case ReedsSheppPathSegment::Direction::BCK:
        return DrivingDirection::Backwards;
      default: { throw std::runtime_error("Unknown conversion"); }
    }
  };

  se2_navigation_msgs::Path pathOut;
  pathOut.segment_.reserve(path.segment_.size());
  for (const auto& segment : path.segment_) {
    se2_navigation_msgs::PathSegment segmentOut;
    segmentOut.points_.reserve(segment.point_.size());
    segmentOut.direction_ = convertDirections(segment.direction_);
    for (const auto& point : segment.point_) {
      segmentOut.points_.push_back(convert(point));
    }
    pathOut.segment_.push_back(segmentOut);
  }

  return pathOut;
}

} /* namespace se2_planning */
