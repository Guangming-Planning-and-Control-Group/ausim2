#include "control/motor_mixer.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>

namespace quadrotor {

MotorMixer::MotorMixer(const MixerParams& params) : params_(params) {
  mat_ << params_.Ct, params_.Ct, params_.Ct, params_.Ct,
      params_.Ct * params_.L, -params_.Ct * params_.L, -params_.Ct * params_.L, params_.Ct * params_.L,
      -params_.Ct * params_.L, -params_.Ct * params_.L, params_.Ct * params_.L, params_.Ct * params_.L,
      -params_.Cd, params_.Cd, -params_.Cd, params_.Cd;
  inv_mat_ = mat_.inverse();
}

Eigen::Vector4d MotorMixer::calculate(double thrust, double mx, double my, double mz) const {
  double Mx = mx;
  double My = my;
  double Mz = 0.0;

  const Eigen::Vector4d control_input(thrust, Mx, My, Mz);
  Eigen::Vector4d motor_speed_sq = inv_mat_ * control_input;

  const double max_value = motor_speed_sq.maxCoeff();
  const double min_value = motor_speed_sq.minCoeff();
  const double ref_value = motor_speed_sq.sum() / 4.0;

  double max_trim_scale = 1.0;
  double min_trim_scale = 1.0;

  if (max_value > params_.max_speed * params_.max_speed) {
    max_trim_scale =
        (params_.max_speed * params_.max_speed - ref_value) / (max_value - ref_value);
  }
  if (min_value < 0.0) {
    min_trim_scale = ref_value / (ref_value - min_value);
  }

  const double scale = std::min(max_trim_scale, min_trim_scale);
  Mx *= scale;
  My *= scale;

  motor_speed_sq = inv_mat_ * Eigen::Vector4d(thrust, Mx, My, Mz);
  if (scale < 1.0) {
    return motor_speed_sq.cwiseAbs().cwiseSqrt();
  }

  Mz = mz;
  Eigen::Vector4d motor_speed_sq_with_z = inv_mat_ * Eigen::Vector4d(thrust, Mx, My, Mz);
  const double z_max_value = motor_speed_sq_with_z.maxCoeff();
  const double z_min_value = motor_speed_sq_with_z.minCoeff();

  Eigen::Index max_index = 0;
  Eigen::Index min_index = 0;
  motor_speed_sq_with_z.maxCoeff(&max_index);
  motor_speed_sq_with_z.minCoeff(&min_index);

  double max_trim_scale_z = 1.0;
  double min_trim_scale_z = 1.0;

  if (z_max_value > params_.max_speed * params_.max_speed) {
    max_trim_scale_z =
        (params_.max_speed * params_.max_speed - motor_speed_sq[max_index]) /
        (z_max_value - motor_speed_sq[max_index]);
  }
  if (z_min_value < 0.0) {
    min_trim_scale_z =
        motor_speed_sq[min_index] / (motor_speed_sq[min_index] - z_min_value);
  }

  const double scale_z = std::min(max_trim_scale_z, min_trim_scale_z);
  Mz *= scale_z;
  motor_speed_sq_with_z = inv_mat_ * Eigen::Vector4d(thrust, Mx, My, Mz);
  return motor_speed_sq_with_z.cwiseAbs().cwiseSqrt();
}

}  // namespace quadrotor
