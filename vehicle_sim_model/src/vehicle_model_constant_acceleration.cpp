
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 */

#include "vehicle_sim_model/vehicle_model_constant_acceleration.h"
#include <algorithm>

VehicleModelConstantAccelTwist::VehicleModelConstantAccelTwist(
  double vx_lim, double wz_lim, double vx_rate, double wz_rate)
  : VehicleModelInterface(5 /* dim x */, 2 /* dim u */)
  , vx_lim_(vx_lim)
  , wz_lim_(wz_lim)
  , vx_rate_(vx_rate)
  , wz_rate_(wz_rate) {}

const double VehicleModelConstantAccelTwist::getX() const
{
  return state_(static_cast<Eigen::Index>(IDX::X));
}
const double VehicleModelConstantAccelTwist::getY() const
{
  return state_(static_cast<Eigen::Index>(IDX::Y));
}
const double VehicleModelConstantAccelTwist::getYaw() const
{
  return state_(static_cast<Eigen::Index>(IDX::YAW));
}
const double VehicleModelConstantAccelTwist::getVx() const
{
  return state_(static_cast<Eigen::Index>(IDX::VX));
}
const double VehicleModelConstantAccelTwist::getWz() const
{
  return state_(static_cast<Eigen::Index>(IDX::WZ));
}
const double VehicleModelConstantAccelTwist::getSteer() const
{
  return 0.0;
}
void VehicleModelConstantAccelTwist::update(const double& dt)
{
  updateRungeKutta(dt, input_);
}
Eigen::VectorXd VehicleModelConstantAccelTwist::calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
  const double vel = state(static_cast<Eigen::Index>(IDX::VX));
  const double angvel = state(static_cast<Eigen::Index>(IDX::WZ));
  const double yaw = state(static_cast<Eigen::Index>(IDX::YAW));
  const double vx_des = std::max(std::min(input(static_cast<Eigen::Index>(IDX_U::VX_DES)), vx_lim_), -vx_lim_);
  const double wz_des = std::max(std::min(input(static_cast<Eigen::Index>(IDX_U::WZ_DES)), wz_lim_), -wz_lim_);
  double vx_rate = 0.0;
  double wz_rate = 0.0;
  if (vx_des > vel)
  {
    vx_rate = vx_rate_;
  }
  else if (vx_des < vel)
  {
    vx_rate = -vx_rate_;
  }

  if (wz_des > angvel)
  {
    wz_rate = wz_rate_;
  }
  else if (wz_des < angvel)
  {
    wz_rate = -wz_rate_;
  }

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(static_cast<Eigen::Index>(IDX::X)) = vel * cos(yaw);
  d_state(static_cast<Eigen::Index>(IDX::Y)) = vel * sin(yaw);
  d_state(static_cast<Eigen::Index>(IDX::YAW)) = angvel;
  d_state(static_cast<Eigen::Index>(IDX::VX)) = vx_rate;
  d_state(static_cast<Eigen::Index>(IDX::WZ)) = wz_rate;

  return d_state;
}
