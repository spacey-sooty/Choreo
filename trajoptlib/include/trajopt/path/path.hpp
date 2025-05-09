// Copyright (c) TrajoptLib contributors

#pragma once

#include <stdint.h>

#include <functional>
#include <vector>

#include "trajopt/constraint/constraint.hpp"
#include "trajopt/util/symbol_exports.hpp"

namespace trajopt {

/**
 * A path waypoint.
 */
struct TRAJOPT_DLLEXPORT Waypoint {
  /// Instantaneous constraints at the waypoint.
  std::vector<Constraint> waypoint_constraints;

  /// Continuous constraints along the segment.
  std::vector<Constraint> segment_constraints;
};

/**
 * A path.
 *
 * @tparam Drivetrain The drivetrain type (e.g., swerve, differential).
 * @tparam Solution The solution type (e.g., swerve, differential).
 */
template <typename Drivetrain, typename Solution>
struct TRAJOPT_DLLEXPORT Path {
  /// Waypoints along the path.
  std::vector<Waypoint> waypoints;

  /// Drivetrain of the robot.
  Drivetrain drivetrain;

  /// A vector of callbacks to be called with the intermediate solution and a
  /// user-specified handle at every iteration of the solver.
  std::vector<std::function<void(const Solution& solution, int64_t handle)>>
      callbacks;
};

}  // namespace trajopt
