/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

/**
 * @file vehicle_model_constant_acceleration.h
 * @brief vehicle model with constant acceleration for velocity & steeiring
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef VEHICLE_SIM_MODEL_VEHICLE_MODEL_CONSTANT_ACCELERATION_H
#define VEHICLE_SIM_MODEL_VEHICLE_MODEL_CONSTANT_ACCELERATION_H

#include "vehicle_sim_model/vehicle_model_interface.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

/**
 * @class vehicle constant acceleration twist model
 * @brief calculate velocity & angular-velocity with constant acceleration
 */
class VehicleModelConstantAccelTwist : public VehicleModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] wz_lim angular velocity limit [m/s]
   * @param [in] vx_rate acceleration for velocity [m/ss]
   * @param [in] wz_rate acceleration for angular velocity [rad/ss]
   */
  VehicleModelConstantAccelTwist(double vx_lim, double wz_lim, double vx_rate, double wz_rate);

private:
  enum class IDX : int
  {
    X = 0,
    Y,
    YAW,
    VX,
    WZ,
  };
  enum class IDX_U : int
  {
    VX_DES = 0,
    WZ_DES,
  };

  const double vx_lim_;   //!< @brief velocity limit
  const double wz_lim_;   //!< @brief angular velocity limit
  const double vx_rate_;  //!< @brief velocity rate
  const double wz_rate_;  //!< @brief angular velocity rate

  /**
   * @brief get vehicle position x
   */
  const double getX() const override;

  /**
   * @brief get vehicle position y
   */
  const double getY() const override;

  /**
   * @brief get vehicle angle yaw
   */
  const double getYaw() const override;

  /**
   * @brief get vehicle velocity vx
   */
  const double getVx() const override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  const double getWz() const override;

  /**
   * @brief get vehicle steering angle
   */
  const double getSteer() const override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double& dt) override;

  /**
   * @brief calculate derivative of states with constant acceleration
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

#endif  // VEHICLE_SIM_MODEL_VEHICLE_MODEL_CONSTANT_ACCELERATION_H
