/*
 * Planner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/State.hpp"

#include <boost/concept_check.hpp>
#include <ompl/geometric/PathGeometric.h>
namespace se2_planning {

class Planner {
 public:
  Planner() = default;
  virtual ~Planner() = default;

  virtual void setStartingState(const State& startingState) = 0;
  virtual void setGoalState(const State& goalState) = 0;
  virtual bool plan() = 0;
  virtual void getPath(Path* path) const = 0;
  //!Eric_Wang:
  virtual ompl::geometric::PathGeometric getOmplPath() const = 0;
  virtual void getOmplPath(ompl::geometric::PathGeometric* omplPath) const = 0;
  virtual void getOmplInterpolatedPath(ompl::geometric::PathGeometric* omplPath, double spatialResolution) const = 0;

  virtual bool reset() { throw std::runtime_error("Planner: reset() not implemented"); }
  virtual bool initialize() { throw std::runtime_error("Planner: initialize() not implemented"); }
  virtual void getStartingState(State* startingState) { throw std::runtime_error("Planner: getStartingState() not implemented"); }

  virtual void getGoalState(State* goalState) { throw std::runtime_error("Planner: getGoalState() not implemented"); }

  template <class T>
  const T* as() const {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
    return static_cast<const T*>(this);
  }

  template <class T>
  T* as() {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
    return static_cast<T*>(this);
  }
};

} /* namespace se2_planning */
