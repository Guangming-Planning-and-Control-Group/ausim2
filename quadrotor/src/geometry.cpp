#include "math/geometry.hpp"

#include <cmath>

namespace quadrotor::math {

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& quaternion) {
  return quaternion.normalized().toRotationMatrix();
}

Eigen::Vector3d veeMap(const Eigen::Matrix3d& matrix) {
  return Eigen::Vector3d(matrix(2, 1), matrix(0, 2), matrix(1, 0));
}

Eigen::Matrix3d hatMap(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
  result(1, 0) = vector.z();
  result(2, 0) = -vector.y();
  result(0, 1) = -vector.z();
  result(2, 1) = vector.x();
  result(0, 2) = vector.y();
  result(1, 2) = -vector.x();
  return result;
}

Eigen::Matrix3d skewSym(const Eigen::Vector3d& vector) {
  return hatMap(vector);
}

Eigen::Matrix3d so3LieToMat(const Eigen::Vector3d& vector) {
  const double theta = vector.norm();
  const double A = theta > 1e-3 ? std::sin(theta) / theta : 1.0;
  const double B = theta > 1e-3 ? (1.0 - std::cos(theta)) / (theta * theta) : 0.5;

  const Eigen::Matrix3d wx = skewSym(vector);
  return Eigen::Matrix3d::Identity() + A * wx + B * wx * wx;
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> se3LieToRotTrans3(
    const Eigen::Vector3d& w,
    const Eigen::Vector3d& u) {
  const double theta = w.norm();
  const double A = theta > 1e-3 ? std::sin(theta) / theta : 1.0;
  const double B = theta > 1e-3 ? (1.0 - std::cos(theta)) / (theta * theta) : 0.5;
  const double C = theta > 1e-3 ? (1.0 - A) / (theta * theta) : 1.0 / 6.0;

  const Eigen::Matrix3d wx = skewSym(w);
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + A * wx + B * wx * wx;
  const Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + B * wx + C * wx * wx;
  return {rotation, V * u};
}

Eigen::Matrix4d se3LieToMat4(const Eigen::Vector3d& w, const Eigen::Vector3d& u) {
  const auto [rotation, translation] = se3LieToRotTrans3(w, u);
  return rotTrans3ToMat4(rotation, translation);
}

Eigen::Matrix4d rotTrans3ToMat4(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

}  // namespace quadrotor::math
