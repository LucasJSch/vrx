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
#include <gz/sim/components/Pose.hh>
#include <sdf/sdf.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

#include "ShapeVolume.hh"
#include "Aerodynamics.hh"

using namespace gz;
using namespace sim;
using namespace vrx;

class vrx::AerodynamicsPrivate
{

  /// \brief Entity of the model attached to the plugin.
  public: sim::Entity entity = sim::kNullEntity;

  /// \brief The link entity.
  public: sim::Link link;

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Object vertices.
  //public: std::vector<gz::math::Vector3d> vertices;

  public: int numVertices;

  public: int airDensity;

  public: ShapeVolumePtr shapeVolumePtr;
};


//////////////////////////////////////////////////
vrx::Aerodynamics::Aerodynamics()
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

  // Parse optional <air_density>.
  if (_sdf->HasElement("air_density"))
    this->dataPtr->airDensity = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("drone_num_vertices"))
    this->dataPtr->numVertices = _sdf->Get<double>("drone_num_vertices");

  this->dataPtr->entity = _entity;

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

  auto element_ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr geometry = element_ptr->GetElement("geometry");
  this->dataPtr->shapeVolumePtr = std::move(ShapeVolume::makeShape(geometry));

  gzdbg << "Aerodynamics plugin successfully configured with the "
        << "following parameters:"                        << std::endl;
  gzdbg << "  <link_name>: " << linkName                  << std::endl;
}

gz::math::Vector3d transformVector(math::Pose3d ref_pose, gz::math::Vector3d vector)
{
  return ref_pose.Rot().Inverse().RotateVector(vector - ref_pose.Pos());
}

Wrench Aerodynamics::getDragWrench(gz::sim::EntityComponentManager &_ecm, const gz::math::Quaterniond& orientation,
                     const gz::math::Vector3d& linear_vel,
                     const gz::math::Vector3d& angular_vel_body,
                     const gz::math::Vector3d& wind_world) const
  {
  //add linear drag due to velocity we had since last dt seconds + wind
  //drag vector magnitude is proportional to v^2, direction opposite of velocity
  //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
  //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
  //http://physics.stackexchange.com/questions/304742/angular-drag-on-body
  //similarly calculate angular drag
  //note that angular velocity, acceleration, torque are already in body frame

  Wrench wrench;
  const float air_density = this->dataPtr->airDensity;
  auto comPose = this->dataPtr->link.WorldInertialPose(_ecm);

  auto modelPose =  _ecm.Component<components::Pose>(this->dataPtr->entity)->Data();

  // Use relative velocity of the body wrt wind
  const gz::math::Vector3d relative_vel = linear_vel - wind_world;
  const gz::math::Vector3d linear_vel_body = modelPose.Rot().Inverse() * relative_vel;

  for (uint vi = 0; vi < this->dataPtr->shapeVolumePtr->getPolyhedron().getAmountOfFaces(); ++vi) {
      auto face_center_pos = transformVector(modelPose, this->dataPtr->shapeVolumePtr->getPolyhedron().getFaceCenterPosition(vi));
      auto face_normal = transformVector(modelPose, this->dataPtr->shapeVolumePtr->getPolyhedron().getFaceNormal(vi));
      const gz::math::Vector3d vel_vertex = linear_vel_body + angular_vel_body.Cross(face_center_pos);
      const float vel_comp = face_normal.Dot(vel_vertex);
      //if vel_comp is -ve then we cull the face. If velocity too low then drag is not generated
      if (vel_comp > 0.1) {
          // TODO(Scheink): Parece ser cte. getDragFactor(), pero doble chequear una vez que funcione todo.
          // https://github.com/microsoft/AirSim/blob/main/AirLib/include/physics/DebugPhysicsBody.hpp#L42C48-L42C48
          const gz::math::Vector3d drag_force = face_normal * (-1.3 * air_density * vel_comp * vel_comp);
          const gz::math::Vector3d drag_torque = face_center_pos.Cross(drag_force);

          wrench.force += drag_force;
          wrench.torque += drag_torque;
      }
  }

  //convert force to world frame, leave torque to local frame
  //wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

  return wrench;
}

Wrench Aerodynamics::getBodyWrench(const gz::math::Quaterniond& orientation) const
{
  //set wrench sum to zero
  Wrench wrench;

  // TODO(scheink): Compute this.
  //calculate total force on rigid body's center of gravity
  /*for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
      //aggregate total
      const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
      const auto& vertex_wrench = vertex.getWrench();
      wrench += vertex_wrench;

      //add additional torque due to force applies farther than COG
      // tau = r X F
      wrench.torque += vertex.getPosition().cross(vertex_wrench.force);
  }*/

  //convert force to world frame, leave torque to local frame
  //wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

  return wrench;
}

/*void Aerodynamics::computeNextPose(TTimeDelta dt, const Pose& current_pose, const gz::math::Vector3d& avg_linear, const gz::math::Vector3d& avg_angular, Kinematics::State& next) const
        {
            float dt_real = static_cast<float>(dt);

            next.pose.position = current_pose.position + avg_linear * dt_real;

            //use angular velocty in body frame to calculate angular displacement in last dt seconds
            float angle_per_unit = avg_angular.norm();
            if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f)) {
                //convert change in angle to unit quaternion
                AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
                Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
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
                next.pose.orientation = current_pose.orientation * angle_dt_q;
                if (VectorMath::hasNan(next.pose.orientation)) {
                    //Utils::DebugBreak();
                    Utils::log("orientation had NaN!", Utils::kLogLevelError);
                }

                //re-normalize quaternion to avoid accumulating error
                next.pose.orientation.normalize();
            }
            else //no change in angle, because angular velocity is zero (normalized vector is undefined)
                next.pose.orientation = current_pose.orientation;
        }*/


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

  /*SCHEINK*/
  //add linear drag due to velocity we had since last dt seconds + wind
  //drag vector magnitude is proportional to v^2, direction opposite of velocity
  //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
  //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
  auto dt_real = _info.dt;
  gz::math::Vector3d avg_linear;
  avg_linear.X() = worldLinearVel->X() + worldLinearAccel->X() * (0.5f * std::chrono::duration<double>(dt_real).count());
  avg_linear.Y() = worldLinearVel->Y() + worldLinearAccel->Y() * (0.5f * std::chrono::duration<double>(dt_real).count());
  avg_linear.Z() = worldLinearVel->Z() + worldLinearAccel->Z() * (0.5f * std::chrono::duration<double>(dt_real).count());
  gz::math::Vector3d avg_angular;
  avg_angular.X() = worldAngularVel->X() + worldAngularAccel->X() * (0.5f * std::chrono::duration<double>(dt_real).count());
  avg_angular.Y() = worldAngularVel->Y() + worldAngularAccel->Y() * (0.5f * std::chrono::duration<double>(dt_real).count());
  avg_angular.Z() = worldAngularVel->Z() + worldAngularAccel->Z() * (0.5f * std::chrono::duration<double>(dt_real).count());
  gz::math::Vector3d wind; // TODO(scheink): Set this
  const Wrench drag_wrench = getDragWrench(_ecm, orientation, avg_linear, avg_angular, wind);
  /*const Wrench body_wrench = getBodyWrench(body, comPose.orientation);
  Wrench next_wrench = body_wrench + drag_wrench;
  Kinematics::State next;
  next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;
  //get new angular acceleration
  //Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
  //we will use torque to find out the angular acceleration
  //angular momentum L = I * omega
  const gz::math::Vector3d angular_momentum = body.getInertia() * avg_angular;
  const gz::math::Vector3d angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
  //new angular acceleration - we'll use this acceleration in next time step
  next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;
  
  // ************************* Update pose and twist after dt ************************ //
  //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
  next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
  next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);
  
  //if controller has bug, velocities can increase idenfinitely
  //so we need to clip this or everything will turn in to infinity/nans
  
  if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
      next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
      next.accelerations.linear = gz::math::Vector3d::Zero();
  }
  //
  //for disc of 1m radius which angular velocity translates to speed of light on tangent?
  if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
      next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
      next.accelerations.angular = gz::math::Vector3d::Zero();
  }
  computeNextPose(dt, current.pose, avg_linear, avg_angular, next);*/
  /*SCHEINK*/



  // Transform the force and torque to the world frame.

  // Apply the force and torque at COM.
  gz::math::Vector3d forceWorld;
  gz::math::Vector3d torqueWorld;
  this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
}

GZ_ADD_PLUGIN(vrx::Aerodynamics,
              sim::System,
              Aerodynamics::ISystemConfigure,
              Aerodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::Aerodynamics,
                    "vrx::Aerodynamics")
