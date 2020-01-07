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
 * @file vehicle_model_ideal.h
 * @brief vehicle ideal velocity model (no dynamics for desired velocity & anguler-velocity or steering)
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef VEHICLE_SIM_MODEL_VEHICLE_MODEL_IDEAL_H
#define VEHICLE_SIM_MODEL_VEHICLE_MODEL_IDEAL_H

#include "vehicle_sim_model/vehicle_model_interface.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

/**
 * @class vehicle ideal twist model
 * @brief calculate ideal twist dynamics
 */
class VehicleModelIdealTwist : public VehicleModelInterface
{
public:
  /**
   * @brief constructor
   */
  VehicleModelIdealTwist();

private:
  enum class IDX : int
  {
    X = 0,
    Y,
    YAW,
  };
  enum class IDX_U : int
  {
    VX_DES = 0,
    WZ_DES,
  };

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
   * @brief calculate derivative of states with ideal twist model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

/**
 * @class vehicle ideal steering model
 * @brief calculate ideal steering dynamics
 */
class VehicleModelIdealSteer : public VehicleModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] wheelbase vehicle wheelbase length [m]
   */
  explicit VehicleModelIdealSteer(double wheelbase);

private:
  enum class IDX : int
  {
    X = 0,
    Y,
    YAW,
  };
  enum class IDX_U : int
  {
    VX_DES = 0,
    STEER_DES,
  };

  const double wheelbase_;  //!< @brief vehicle wheelbase length

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
   * @brief calculate derivative of states with ideal steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

#endif  // VEHICLE_SIM_MODEL_VEHICLE_MODEL_IDEAL_H
