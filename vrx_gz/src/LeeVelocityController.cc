/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "LeeVelocityController.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
namespace systems
{
namespace multicopter_control
{
//////////////////////////////////////////////////
std::unique_ptr<LeeVelocityController> LeeVelocityController::MakeController(
    const LeeVelocityControllerParameters &_controllerParams,
    const VehicleParameters &_vehicleParams)
{
  // auto controller = std::make_unique<LeeVelocityController>();
  // Can't use make_unique here because the constructor is private
  std::unique_ptr<LeeVelocityController> controller(
      new LeeVelocityController());
  controller->controllerParameters = _controllerParams;
  controller->vehicleParameters = _vehicleParams;
  if (controller->InitializeParameters())
  {
    return controller;
  }
  else
  {
    return nullptr;
  }
}

//////////////////////////////////////////////////
bool LeeVelocityController::InitializeParameters()
{
  auto allocationMatrix =
      calculateAllocationMatrix(this->vehicleParameters.rotorConfiguration);
  if (!allocationMatrix.has_value())
  {
    // Error should already be printed by function
    return false;
  }

  // To make the tuning independent of the inertia matrix we divide here.
  this->normalizedAttitudeGain =
      this->controllerParameters.attitudeGain.transpose() *
      this->vehicleParameters.inertia.inverse();

  this->normalizedAngularRateGain =
      this->controllerParameters.angularRateGain.transpose() *
      this->vehicleParameters.inertia.inverse();

  Eigen::Matrix4d moi;
  moi.setZero();
  moi.block<3, 3>(0, 0) = this->vehicleParameters.inertia;
  moi(3, 3) = 1;

  this->angularAccToRotorVelocities.resize(
      this->vehicleParameters.rotorConfiguration.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia
  // matrix I. A^{ \dagger} = A^T*(A*A^T)^{-1}
  const auto &aMat = *allocationMatrix;
  this->angularAccToRotorVelocities =
      aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;

  return true;
}

double LeeVelocityController::Gamma(double k_alpha, double sigma, double k_beta)
{
  //return -k_alpha * sqrt(abs(sigma)) * sign(sigma) - k_beta * sigma;
  return 0;
}

//////////////////////////////////////////////////
void LeeVelocityController::CalculateRotorVelocities(
    const FrameData &_frameData, const EigenTwist &_cmdVel, const EigenTwist &_cmdAccel, const Eigen::Isometry3d &_desiredPose,
    Eigen::VectorXd &_rotorVelocities) const
{
  double LAMBDA_C = 0.001;
  double K_BETA = 0.02;
  double K_MIN = 0.05;
  double MU = 0.05;
  double K_R = 0.1;

  std::vector<double> k_beta(7, K_BETA);
  std::vector<double> lambda_c(7, LAMBDA_C);
  std::vector<double> k_min_arr(7, K_MIN);;
  std::vector<double> k_r_arr(7, K_R);
  std::vector<double> mu(7, MU);

  // Rcontrol
  /*Lambda, mu, p, r_cluster, v_cluster = self.modelo.modelo(
      r_asv, r_uav, v_asv, v_uav
  )
  // Error en velocidad
  auto error_c_dot = _frameData.linearVelocityWorld - _cmdVel.linear;
  // Error en posición
  auto error_c = _frameData.pose.linear() - _desiredPose.linear();
  // Superficie de deslizamiento
  double lambda_c = 0.001;
  auto superficie = error_c_dot + lambda_c * error_c;
  // Ganancia adaptativa
  auto k_alpha = adaptive_gain(t, superficie);
  auto gamma = Gamma(k_alpha, superficie, k_beta);
  // Feedback
  auto F = Lambda.dot(_cmdAccel.linear - gamma + lambda_c * error_c_dot) + mu + p;
  // Cluster Space -> Robot Space
  auto G = self.modelo.kinematics.jacobian(r_asv, r_uav).T.dot(F);*/

  /*_rotorVelocities =
      _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));
  _rotorVelocities = _rotorVelocities.cwiseSqrt();*/
}

//////////////////////////////////////////////////
Eigen::Vector3d LeeVelocityController::ComputeDesiredAcceleration(
                 const FrameData &_frameData, const EigenTwist &_cmdVel) const
{
  Eigen::Vector3d velocityError = _frameData.linearVelocityWorld -
                                  _frameData.pose.linear() * _cmdVel.linear;

  Eigen::Vector3d accelCommand =
      velocityError.cwiseProduct(this->controllerParameters.velocityGain) /
      this->vehicleParameters.mass;

  accelCommand = accelCommand.cwiseAbs()
      .cwiseMin(this->controllerParameters.maxLinearAcceleration)
      .cwiseProduct(accelCommand.cwiseSign());

  return accelCommand + this->vehicleParameters.gravity;
}

//////////////////////////////////////////////////
// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on
// SE(3)
Eigen::Vector3d LeeVelocityController::ComputeDesiredAngularAcc(
    const FrameData &_frameData, const EigenTwist &_cmdVel,
    const Eigen::Vector3d &_acceleration) const
{
  const Eigen::Matrix3d& rot = _frameData.pose.linear();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1Des = rot.col(0);

  Eigen::Vector3d b3Des;
  b3Des = -_acceleration / _acceleration.norm();

  // Check if b1 and b3 are parallel. If so, choose a different b1 vector. This
  // could happen if the UAV is rotated by 90 degrees w.r.t the horizontal
  // plane.
  const double tol = 1e-3;
  if (b1Des.cross(b3Des).squaredNorm() < tol)
  {
    // acceleration and b1 are parallel. Choose a different vector
    b1Des = rot.col(1);

    if (b1Des.cross(b3Des).squaredNorm() < tol)
    {
      b1Des = rot.col(2);
    }
  }

  Eigen::Vector3d b2Des;
  b2Des = b3Des.cross(b1Des);
  b2Des.normalize();

  Eigen::Matrix3d rotDes;
  rotDes.col(0) = b2Des.cross(b3Des);
  rotDes.col(1) = b2Des;
  rotDes.col(2) = b3Des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angleErrorMatrix =
      0.5 * (rotDes.transpose() * rot - rot.transpose() * rotDes);
  Eigen::Vector3d angleError = vectorFromSkewMatrix(angleErrorMatrix);

  Eigen::Vector3d angularRateDes(Eigen::Vector3d::Zero());
  angularRateDes[2] = _cmdVel.angular[2];

  // The paper shows
  // e_omega = omega - R.T * R_d * omega_des
  // The code in the RotorS implementation has
  // e_omega = omega - R_d.T * R * omega_des
  Eigen::Vector3d angularRateError = _frameData.angularVelocityBody -
                                     rot.transpose() * rotDes * angularRateDes;

  // The following MOI terms are computed in the paper, but the RotorS
  // implementation ignores them. They don't appear to make much of a
  // difference.
  // Eigen::Matrix3d moi = this->vehicleParameters.inertia;
  // const Eigen::Vector3d &omega = _frameData.angularVelocityBody;

  // Eigen::Vector3d moiTerm = omega.cross(moi * omega);

  // Eigen::Vector3d moiTerm2 = moi * (skewMatrixFromVector(omega) *
  //                            rot.transpose() * rotDes * angularRateDes);

  // std::cout << moiTerm2.transpose() << std::endl;
  // return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
  //         angularRateError.cwiseProduct(this->normalizedAngularRateGain) +
  //         moiTerm - moiTerm2;
  return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
          angularRateError.cwiseProduct(this->normalizedAngularRateGain);
}
}  // namespace multicopter_control
}  // namespace systems
}  // namespace sim
}  // namespace gz