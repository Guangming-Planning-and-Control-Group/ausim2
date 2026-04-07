#include "controller/se3_controller.hpp"

#include <stdexcept>

#include "math/geometry.hpp"

namespace quadrotor {

void SE3Controller::setCurrentState(const State& state) {
  current_state_ = state;
}

void SE3Controller::setGoalState(const State& state) {
  goal_state_ = state;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SE3Controller::updateLinearError() const {
  if (!goal_state_.has_value() || !current_state_.has_value()) {
    throw std::runtime_error("SE3Controller state is not initialized.");
  }

  const Eigen::Vector3d e_x = current_state_->position - goal_state_->position;
  const Eigen::Vector3d e_v = current_state_->velocity - goal_state_->velocity;
  return {e_x, e_v};
}

SE3Controller::AngularError SE3Controller::updateAngularError(
    const Eigen::Vector3d& trans_control,
    const Eigen::Vector3d& forward) const {
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
    forward_des = R_curr.col(1);
  }

  Eigen::Vector3d right_des = forward_des.cross(goal_z);
  if (right_des.norm() < 1e-6) {
    const Eigen::Vector3d fallback =
        std::abs(goal_z.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    right_des = fallback.cross(goal_z);
  }
  right_des.normalize();

  Eigen::Vector3d proj_fwd_des = goal_z.cross(right_des);
  proj_fwd_des.normalize();

  Eigen::Matrix3d R_goal = Eigen::Matrix3d::Zero();
  R_goal.col(0) = right_des;
  R_goal.col(1) = proj_fwd_des;
  R_goal.col(2) = goal_z;

  AngularError error;
  error.thrust = goal_z_norm;
  error.attitude =
      0.5 * math::veeMap(R_goal.transpose() * R_curr - R_curr.transpose() * R_goal);
  error.angular_rate =
      current_state_->omega - R_curr.transpose() * R_goal * goal_state_->omega;
  return error;
}

ControlCommand SE3Controller::controlUpdate(
    const State& current_state,
    const State& goal_state,
    double dt,
    const Eigen::Vector3d& forward) {
  (void)dt;
  current_state_ = current_state;
  goal_state_ = goal_state;

  const auto [e_x, e_v] = updateLinearError();
  const Eigen::Vector3d position_error =
      control_mode == ControlMode::kPosition ? e_x : Eigen::Vector3d::Zero();

  const Eigen::Vector3d trans_control(
      -kx * position_error.x() - kv * e_v.x() + goal_state_->velocity.x(),
      -kx * position_error.y() - kv * e_v.y() + goal_state_->velocity.y(),
      -kx * position_error.z() - kv * e_v.z() + goal_state_->velocity.z());

  const AngularError angular_error = updateAngularError(trans_control, forward);

  ControlCommand command;
  command.thrust = angular_error.thrust;
  command.angular = Eigen::Vector3d(
      -kR * angular_error.attitude.x() - kw * angular_error.angular_rate.x() + goal_state_->omega.x(),
      -kR * angular_error.attitude.y() - kw * angular_error.angular_rate.y() + goal_state_->omega.y(),
      -kR * angular_error.attitude.z() - kw * angular_error.angular_rate.z() + goal_state_->omega.z());
  return command;
}

}  // namespace quadrotor
