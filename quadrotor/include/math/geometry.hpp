#pragma once

#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace quadrotor::math {

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& quaternion);
Eigen::Vector3d veeMap(const Eigen::Matrix3d& matrix);
Eigen::Matrix3d hatMap(const Eigen::Vector3d& vector);
Eigen::Matrix3d skewSym(const Eigen::Vector3d& vector);
Eigen::Matrix3d so3LieToMat(const Eigen::Vector3d& vector);
std::pair<Eigen::Matrix3d, Eigen::Vector3d> se3LieToRotTrans3(
    const Eigen::Vector3d& w,
    const Eigen::Vector3d& u);
Eigen::Matrix4d se3LieToMat4(const Eigen::Vector3d& w, const Eigen::Vector3d& u);
Eigen::Matrix4d rotTrans3ToMat4(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

}  // namespace quadrotor::math
