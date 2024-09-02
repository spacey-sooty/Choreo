// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/geometry/Line.hpp"
#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

class TRAJOPT_DLLEXPORT LaneConstraint {
 public:
  LaneConstraint(Line<double> line1, Line<double> line2)
      : m_line1(std::move(line1)), m_line2(std::move(line2)) {}

  /**
   * Applies this constraint to the given problem.
   *
   * @param problem The optimization problem.
   * @param pose The robot's pose.
   * @param linearVelocity The robot's linear velocity.
   * @param angularVelocity The robot's angular velocity.
   * @param linearAcceleration The robot's linear acceleration.
   * @param angularAcceleration The robot's angular acceleration.
   */
  void Apply(sleipnir::OptimizationProblem& problem, const Pose2v& pose,
             [[maybe_unused]] const Translation2v& linearVelocity,
             [[maybe_unused]] const sleipnir::Variable& angularVelocity,
             [[maybe_unused]] const Translation2v& linearAcceleration,
             [[maybe_unused]] const sleipnir::Variable& angularAcceleration) {
    // assuming robot is 0 size point
    // Line Constraint:
    // ax + by > c

    // m = (y₁ - y₀)/(x₁ - x₀)
    // y - y₀ = m(x - x₀)

    // y - y₀ = mx - mx₀
    // y = mx - mx₀ + y₀
    // -mx + y = -mx₀ + y₀
    // -(y₁ - y₀)/(x₁ - x₀)x + y = -(y₁ - y₀)/(x₁ - x₀)x₀ + y₀
    // -(y₁ - y₀)x + (x₁ - x₀)y = -(y₁ - y₀)x₀ + (x₁ - x₀)y₀
    // (y₀ - y₁)x + (x₁ - x₀)y = (y₀ - y₁)x₀ + (x₁ - x₀)y₀
    // (y₀ - y₁)x + (x₁ - x₀)y > (y₀ - y₁)x₀ + (x₁ - x₀)y₀

    auto a = (m_line1.startPoint.Y() - m_line1.endPoint.Y());
    auto b = (m_line1.endPoint.X() - m_line1.startPoint.X());
    auto c = a * m_line1.startPoint.X() + b * m_line1.startPoint.Y();
    problem.SubjectTo(a * pose.X() + b * pose.Y() > c);
  }

 private:
  Line<double> m_line1;
  Line<double> m_line2;
};

}  // namespace trajopt
