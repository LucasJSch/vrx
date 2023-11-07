/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <string>
#include <Eigen/Eigen>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

#include "Aerodynamics.hh"

using namespace gz;
using namespace sim;
using namespace vrx;

class vrx::AerodynamicsPrivate
{
  /// \brief The link entity.
  public: sim::Link link;

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Added mass in surge, X_\dot{u}.
  public: double paramXdotU{0.0};

  /// \brief Added mass in sway, Y_\dot{v}.
  public: double paramYdotV{0.0};

  /// \brief Added mass in heave, Z_\dot{w}.
  public: double paramZdotW{0.0};

  /// \brief Added mass in roll, K_\dot{p}.
  public: double paramKdotP{0.0};

  /// \brief Added mass in pitch, M_\dot{q}.
  public: double paramMdotQ{0.0};

  /// \brief Added mass in yaw, N_\dot{r}.
  public: double paramNdotR{0.0};

  /// \brief Linear drag in surge.
  public: double paramXu{0.0};

  /// \brief Quadratic drag in surge.
  public: double paramXuu{0.0};

  /// \brief Linear drag in sway.
  public: double paramYv{0.0};

  /// \brief Quadratic drag in sway.
  public: double paramYvv{0.0};

  /// \brief Linear drag in heave.
  public: double paramZw{0.0};

  /// \brief Quadratic drag in heave.
  public: double paramZww{0.0};

  /// \brief Linear drag in roll.
  public: double paramKp{0.0};

  /// \brief Quadratic drag in roll.
  public: double paramKpp{0.0};

  /// \brief Linear drag in pitch.
  public: double paramMq{0.0};

  /// \brief Quadratic drag in pitch.
  public: double paramMqq{0.0};

  /// \brief Linear drag in yaw.
  public: double paramNr{0.0};

  /// \brief Quadratic drag in yaw.
  public: double paramNrr{0.0};

  /// \brief Added mass of vehicle.
  /// See: https://en.wikipedia.org/wiki/Added_mass
  public: Eigen::MatrixXd Ma;
};


//////////////////////////////////////////////////
Aerodynamics::Aerodynamics()
  : dataPtr(std::make_unique<AerodynamicsPrivate>())
{
}

//////////////////////////////////////////////////
void Aerodynamics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  this->dataPtr->link.EnableVelocityChecks(_ecm);
  this->dataPtr->link.EnableAccelerationChecks(_ecm);

  this->dataPtr->paramXdotU       = _sdf->Get<double>("xDotU", 5  ).first;
  this->dataPtr->paramYdotV       = _sdf->Get<double>("yDotV", 5  ).first;
  this->dataPtr->paramZdotW       = _sdf->Get<double>("zDotW", 0.1).first;
  this->dataPtr->paramKdotP       = _sdf->Get<double>("kDotP", 0.1).first;
  this->dataPtr->paramMdotQ       = _sdf->Get<double>("mDotQ", 0.1).first;
  this->dataPtr->paramNdotR       = _sdf->Get<double>("nDotR", 1  ).first;
  this->dataPtr->paramXu          = _sdf->Get<double>("xU",   20  ).first;
  this->dataPtr->paramXuu         = _sdf->Get<double>("xUU",   0  ).first;
  this->dataPtr->paramYv          = _sdf->Get<double>("yV",   20  ).first;
  this->dataPtr->paramYvv         = _sdf->Get<double>("yVV",   0  ).first;
  this->dataPtr->paramZw          = _sdf->Get<double>("zW",   20  ).first;
  this->dataPtr->paramZww         = _sdf->Get<double>("zWW",   0  ).first;
  this->dataPtr->paramKp          = _sdf->Get<double>("kP",   20  ).first;
  this->dataPtr->paramKpp         = _sdf->Get<double>("kPP",   0  ).first;
  this->dataPtr->paramMq          = _sdf->Get<double>("mQ",   20  ).first;
  this->dataPtr->paramMqq         = _sdf->Get<double>("mQQ",   0  ).first;
  this->dataPtr->paramNr          = _sdf->Get<double>("nR",   20  ).first;
  this->dataPtr->paramNrr         = _sdf->Get<double>("nRR",   0  ).first;

  // Added mass according to Fossen's equations (p 37).
  this->dataPtr->Ma = Eigen::MatrixXd::Zero(6, 6);

  this->dataPtr->Ma(0, 0) = this->dataPtr->paramXdotU;
  this->dataPtr->Ma(1, 1) = this->dataPtr->paramYdotV;
  this->dataPtr->Ma(2, 2) = this->dataPtr->paramZdotW;
  this->dataPtr->Ma(3, 3) = this->dataPtr->paramKdotP;
  this->dataPtr->Ma(4, 4) = this->dataPtr->paramMdotQ;
  this->dataPtr->Ma(5, 5) = this->dataPtr->paramNdotR;

  gzdbg << "Aerodynamics plugin successfully configured with the "
        << "following parameters:"                        << std::endl;
  gzdbg << "  <link_name>: " << linkName                  << std::endl;
  gzdbg << "  <xDotU>: "     << this->dataPtr->paramXdotU << std::endl;
  gzdbg << "  <yDotV>: "     << this->dataPtr->paramYdotV << std::endl;
  gzdbg << "  <zDotW>: "     << this->dataPtr->paramZdotW << std::endl;
  gzdbg << "  <kDotP>: "     << this->dataPtr->paramKdotP << std::endl;
  gzdbg << "  <mDotQ>: "     << this->dataPtr->paramMdotQ << std::endl;
  gzdbg << "  <nDotR>: "     << this->dataPtr->paramNdotR << std::endl;
  gzdbg << "  <xU>: "        << this->dataPtr->paramXu    << std::endl;
  gzdbg << "  <xUU>: "       << this->dataPtr->paramXuu   << std::endl;
  gzdbg << "  <yV>: "        << this->dataPtr->paramYv    << std::endl;
  gzdbg << "  <yVV>: "       << this->dataPtr->paramYvv   << std::endl;
  gzdbg << "  <zW>: "        << this->dataPtr->paramZw    << std::endl;
  gzdbg << "  <zWW>: "       << this->dataPtr->paramZww   << std::endl;
  gzdbg << "  <kP>: "        << this->dataPtr->paramKp    << std::endl;
  gzdbg << "  <kPP>: "       << this->dataPtr->paramKpp   << std::endl;
  gzdbg << "  <mQ>: "        << this->dataPtr->paramMq    << std::endl;
  gzdbg << "  <mQQ>: "       << this->dataPtr->paramMqq   << std::endl;
  gzdbg << "  <nR>: "        << this->dataPtr->paramNr    << std::endl;
  gzdbg << "  <nRR>: "       << this->dataPtr->paramNrr   << std::endl;
}

//////////////////////////////////////////////////
void Aerodynamics::PreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("Aerodynamics::PreUpdate");

  if (_info.paused)
    return;

  if (!this->dataPtr->link.Valid(_ecm))
    return;

  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);

  // Get vehicle state.
  auto worldAngularVel = this->dataPtr->link.WorldAngularVelocity(_ecm);
  auto worldLinearVel = this->dataPtr->link.WorldLinearVelocity(_ecm);
  auto worldAngularAccel = this->dataPtr->link.WorldAngularAcceleration(_ecm);
  auto worldLinearAccel = this->dataPtr->link.WorldLinearAcceleration(_ecm);

  // Sanity check: Make sure that we can read the full state.
  if (!worldAngularVel)
  {
    gzerr << "No angular velocity" <<"\n";
    return;
  }

  if (!worldLinearVel)
  {
    gzerr << "No linear velocity" <<"\n";
    return;
  }

  if (!worldAngularAccel)
  {
    gzerr << "No angular acceleration" <<"\n";
    return;
  }

  if (!worldLinearAccel)
  {
    gzerr << "No linear acceleration" <<"\n";
    return;
  }

  // Transform from world to local frame.
  auto comPose = this->dataPtr->link.WorldInertialPose(_ecm);
  auto localAngularVel   = comPose->Rot().Inverse() * (*worldAngularVel);
  auto localLinearVel    = comPose->Rot().Inverse() * (*worldLinearVel);
  auto localAngularAccel = comPose->Rot().Inverse() * (*worldAngularAccel);
  auto localLinearAccel  = comPose->Rot().Inverse() * (*worldLinearAccel);

  stateDot << localLinearAccel.X(), localLinearAccel.Y(), localLinearAccel.Z(),
   localAngularAccel.X(), localAngularAccel.Y(), localAngularAccel.Z();

  state << localLinearVel.X(), localLinearVel.Y(), localLinearVel.Z(),
    localAngularVel.X(), localAngularVel.Y(), localAngularVel.Z();

  // Added Mass.
  const Eigen::VectorXd kAmassVec = -1.0 * this->dataPtr->Ma * stateDot;

  /*SCHEINK*/
  //add linear drag due to velocity we had since last dt seconds + wind
  //drag vector magnitude is proportional to v^2, direction opposite of velocity
  //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
  //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
  auto dt_real = _info.dt;
  avg_linear = *worldLinearVel + (*worldLinearAccel) * (0.5f * dt_real);
  avg_angular = *worldAngularVel + (*worldAngularAccel) * (0.5f * dt_real);
  Vector3r& wind = Vector3r(); // TODO: Set this
  const Wrench drag_wrench = getDragWrench(body, comPose.orientation, avg_linear, avg_angular, wind);
  const Wrench body_wrench = getBodyWrench(body, comPose.orientation);
  Wrench next_wrench = body_wrench + drag_wrench;
  Kinematics::State next;
  next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;
  //get new angular acceleration
  //Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
  //we will use torque to find out the angular acceleration
  //angular momentum L = I * omega
  const Vector3r angular_momentum = body.getInertia() * avg_angular;
  const Vector3r angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
  //new angular acceleration - we'll use this acceleration in next time step
  next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;
  
  /************************* Update pose and twist after dt ************************/
  //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
  next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
  next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);
  
  //if controller has bug, velocities can increase idenfinitely
  //so we need to clip this or everything will turn in to infinity/nans
  
  if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
      next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
      next.accelerations.linear = Vector3r::Zero();
  }
  //
  //for disc of 1m radius which angular velocity translates to speed of light on tangent?
  if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
      next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
      next.accelerations.angular = Vector3r::Zero();
  }
  real_T dt_real = static_cast<real_T>(dt);

  next.pose.position = current_pose.position + avg_linear * dt_real;
  
  //use angular velocty in body frame to calculate angular displacement in last dt seconds
  real_T angle_per_unit = avg_angular.norm();
  if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f)) {
    //convert change in angle to unit quaternion
    AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
    Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
    /*
  Add change in angle to previous orientation.
  Proof that this is q0 * q1:
  If rotated vector is qx*v*qx' then qx is attitude
  Initially we have q0*v*q0'
  Lets transform this to body coordinates to get
  q0'*(q0*v*q0')*q0
  Then apply q1 rotation on it to get
  q1(q0'*(q0*v*q0')*q0)q1'
  Then transform back to world coordinate
  q0(q1(q0'*(q0*v*q0')*q0)q1')q0'
  which simplifies to
  q0(q1(v)q1')q0'
  Thus new attitude is q0q1
  */
      next.pose.orientation = current_pose.orientation * angle_dt_q;
      if (VectorMath::hasNan(next.pose.orientation)) {
          //Utils::DebugBreak();
          Utils::log("orientation had NaN!", Utils::kLogLevelError);
      }
  
      //re-normalize quaternion to avoid accumulating error
      next.pose.orientation.normalize();
  }
  else{
    //no change in angle, because angular velocity is zero (normalized vector is undefined)
    next.pose.orientation = current_pose.orientation;
  }
  /*SCHEINK*/



  // Transform the force and torque to the world frame.

  // Apply the force and torque at COM.
  this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
}

GZ_ADD_PLUGIN(vrx::Aerodynamics,
              sim::System,
              Aerodynamics::ISystemConfigure,
              Aerodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::Aerodynamics,
                    "vrx::Aerodynamics")
