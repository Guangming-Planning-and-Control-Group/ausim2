#pragma once

#include <Eigen/Core>

namespace quadrotor {

struct MixerParams {
  double Ct = 3.25e-4;
  double Cd = 7.9379e-6;
  double L = 0.065 / 2.0;
  double max_thrust = 0.1573;
  double max_torque = 3.842e-03;
  double max_speed = 22.0;
};

class MotorMixer {
 public:
  explicit MotorMixer(const MixerParams& params = MixerParams{});

  Eigen::Vector4d calculate(double thrust, double mx, double my, double mz) const;
  const MixerParams& params() const { return params_; }

 private:
  MixerParams params_;
  Eigen::Matrix4d mat_ = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d inv_mat_ = Eigen::Matrix4d::Zero();
};

}  // namespace quadrotor
