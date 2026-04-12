#include "controller/se3_controller.hpp"

#include <cmath>
#include <stdexcept>

#include "math/geometry.hpp"

namespace quadrotor {

namespace {

Eigen::Matrix3d BuildBodyFromAircraft(const Eigen::Vector3d& aircraft_forward_axis) {
  const Eigen::Vector3d forward_body = aircraft_forward_axis.normalized();
  const Eigen::Vector3d left_body = Eigen::Vector3d::UnitZ().cross(forward_body).normalized();
  const Eigen::Vector3d up_body = forward_body.cross(left_body).normalized();

  Eigen::Matrix3d body_from_aircraft;
  body_from_aircraft.col(0) = forward_body;
  body_from_aircraft.col(1) = left_body;
  body_from_aircraft.col(2) = up_body;
  return body_from_aircraft;
}

Eigen::Vector3d ProjectOntoPlane(const Eigen::Vector3d& vector, const Eigen::Vector3d& normal) { return vector - normal * normal.dot(vector); }

}  // namespace

void SE3Controller::setCurrentState(const State& state) { current_state_ = state; }

void SE3Controller::setGoalState(const State& state) { goal_state_ = state; }

void SE3Controller::setAircraftForwardAxis(const Eigen::Vector3d& axis) {
  if (axis.head<2>().norm() < 1e-6 || std::abs(axis.z()) > 1e-6) {
    throw std::runtime_error("aircraft forward axis must be horizontal and have a non-zero xy component.");
  }
  body_from_aircraft_ = BuildBodyFromAircraft(axis);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SE3Controller::updateLinearError() const {
  if (!goal_state_.has_value() || !current_state_.has_value()) {
    throw std::runtime_error("SE3Controller state is not initialized.");
  }

  const Eigen::Vector3d e_x = current_state_->position - goal_state_->position;
  const Eigen::Vector3d e_v = current_state_->velocity - goal_state_->velocity;
  return {e_x, e_v};
}

SE3Controller::AngularError SE3Controller::updateAngularError(const Eigen::Vector3d& trans_control, const Eigen::Vector3d& forward) const {
  if (!goal_state_.has_value() || !current_state_.has_value()) {
    throw std::runtime_error("SE3Controller state is not initialized.");
  }

  const Eigen::Matrix3d R_curr = math::quaternionToRotationMatrix(current_state_->quaternion);

  Eigen::Vector3d goal_z = trans_control - gravity;
  const double goal_z_norm = goal_z.norm();
  if (goal_z_norm > 1e-6) {
    goal_z /= goal_z_norm;
  } else {
    goal_z = R_curr.col(2);
  }

  Eigen::Vector3d forward_des = forward;
  if (forward_des.norm() > 1e-6) {
    forward_des.normalize();
  } else {
    forward_des = R_curr * body_from_aircraft_.col(0);
  }

  Eigen::Vector3d proj_fwd_des = ProjectOntoPlane(forward_des, goal_z);
  if (proj_fwd_des.norm() < 1e-6) {
    proj_fwd_des = ProjectOntoPlane(R_curr * body_from_aircraft_.col(0), goal_z);
  }
  if (proj_fwd_des.norm() < 1e-6) {
    const Eigen::Vector3d fallback = std::abs(goal_z.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    proj_fwd_des = ProjectOntoPlane(fallback, goal_z);
  }
  proj_fwd_des.normalize();
  Eigen::Vector3d left_des = goal_z.cross(proj_fwd_des);
  left_des.normalize();

  Eigen::Matrix3d R_goal_aircraft = Eigen::Matrix3d::Zero();
  R_goal_aircraft.col(0) = proj_fwd_des;
  R_goal_aircraft.col(1) = left_des;
  R_goal_aircraft.col(2) = goal_z;
  const Eigen::Matrix3d R_goal = R_goal_aircraft * body_from_aircraft_.transpose();

  AngularError error;
  error.thrust = goal_z_norm;
  error.attitude = 0.5 * math::veeMap(R_goal.transpose() * R_curr - R_curr.transpose() * R_goal);
  error.angular_rate = current_state_->omega - R_curr.transpose() * R_goal * goal_state_->omega;
  return error;
}

ControlCommand SE3Controller::controlUpdate(const State& current_state, const State& goal_state, double dt, const Eigen::Vector3d& forward) {
  (void)dt;
  current_state_ = current_state;
  goal_state_ = goal_state;

  const auto [e_x, e_v] = updateLinearError();
  const Eigen::Vector3d position_error = control_mode == ControlMode::kPosition ? e_x : Eigen::Vector3d::Zero();

  const Eigen::Vector3d trans_control(-kx * position_error.x() - kv * e_v.x() + goal_state_->velocity.x(),
                                      -kx * position_error.y() - kv * e_v.y() + goal_state_->velocity.y(),
                                      -kx * position_error.z() - kv * e_v.z() + goal_state_->velocity.z());

  const AngularError angular_error = updateAngularError(trans_control, forward);

  ControlCommand command;
  command.thrust = angular_error.thrust;
  command.angular = Eigen::Vector3d(-kR * angular_error.attitude.x() - kw * angular_error.angular_rate.x() + goal_state_->omega.x(),
                                    -kR * angular_error.attitude.y() - kw * angular_error.angular_rate.y() + goal_state_->omega.y(),
                                    -kR * angular_error.attitude.z() - kw * angular_error.angular_rate.z() + goal_state_->omega.z());
  return command;
}

}  // namespace quadrotor
