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

#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>

#include "vehicle_sim_model/vehicle_model_interface.h"
#include "vehicle_sim_model/vehicle_model_ideal.h"
#include "vehicle_sim_model/vehicle_model_time_delay.h"
#include "vehicle_sim_model/vehicle_model_constant_acceleration.h"

enum class VehicleModelType
{
  IDEAL_TWIST = 0,
  IDEAL_STEER = 1,
  DELAY_TWIST = 2,
  DELAY_STEER = 3,
  CONST_ACCEL_TWIST = 4
};

class TestSuite : public ::testing::Test
{
public:
  VehicleModelType vehicle_model_type_;
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;
  const double wheelbase_;
  const double dt_, angvel_lim_, vel_lim_, steer_lim_, accel_rate_, angvel_rate_,
    steer_rate_lim_, vel_time_delay_, vel_time_constant_, steer_time_delay_,
    steer_time_constant_, angvel_time_delay_, angvel_time_constant_;

  TestSuite()
    : wheelbase_(2.7)
    , dt_(0.02)
    , angvel_lim_(3.0)
    , vel_lim_(10.0)
    , steer_lim_(3.14 / 3.0)
    , accel_rate_(1.0)
    , angvel_rate_(1.0)
    , steer_rate_lim_(0.3)
    , vel_time_delay_(0.25)
    , vel_time_constant_(0.6197)
    , steer_time_delay_(0.24)
    , steer_time_constant_(0.27)
    , angvel_time_delay_(0.2)
    , angvel_time_constant_(0.5) {}
  ~TestSuite()
  {
  }

  void resetVehicleModel()
  {
    vehicle_model_ptr_.reset();
    if (vehicle_model_type_ == VehicleModelType::IDEAL_TWIST)
    {
      vehicle_model_ptr_ = std::make_shared<VehicleModelIdealTwist>();
    }
    else if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER)
    {
      vehicle_model_ptr_ = std::make_shared<VehicleModelIdealSteer>(wheelbase_);
    }
    else if (vehicle_model_type_ == VehicleModelType::DELAY_TWIST)
    {
      vehicle_model_ptr_ = std::make_shared<VehicleModelTimeDelayTwist>(
        vel_lim_, angvel_lim_, accel_rate_, angvel_rate_, dt_, vel_time_delay_,
        vel_time_constant_, angvel_time_delay_, angvel_time_constant_);
    }
    else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER)
    {
      vehicle_model_ptr_ = std::make_shared<VehicleModelTimeDelaySteer>(
        vel_lim_, steer_lim_, accel_rate_, steer_rate_lim_,
        wheelbase_, dt_, vel_time_delay_, vel_time_constant_,
        steer_time_delay_, steer_time_constant_);
    }
    else if (vehicle_model_type_ == VehicleModelType::CONST_ACCEL_TWIST)
    {
      vehicle_model_ptr_ = std::make_shared<VehicleModelConstantAccelTwist>(
        vel_lim_, angvel_lim_, accel_rate_, angvel_rate_);
    }


    if (vehicle_model_type_ == VehicleModelType::IDEAL_TWIST ||
      vehicle_model_type_ == VehicleModelType::IDEAL_STEER)
    {
      Eigen::VectorXd state(3);
      state << 0.0, 0.0, 0.0;
      vehicle_model_ptr_->setState(state);
    }
    else
    {
      Eigen::VectorXd state(5);
      state << 0.0, 0.0, 0.0, 0.0, 0.0;
      vehicle_model_ptr_->setState(state);
    }
  }

  void updateNLoopWithConstantInput(
    const Eigen::VectorXd& input, const int loop_count)
  {
    for (int i = 0; i < loop_count; ++i)
    {
      vehicle_model_ptr_->setInput(input);
      vehicle_model_ptr_->update(dt_);
    }
  }

  void checkVel(const double vel)
  {
    if (vel > 0.0)  // Forward
    {
      ASSERT_GT(vehicle_model_ptr_->getVx(), vel)
        << "Forward: It should be greater than " << vel;
    }
    else if (vel < 0.0)
    {
      ASSERT_LT(vehicle_model_ptr_->getVx(), vel)
        << "Backward: It should be less than " << vel;
    }
  }

  void checkStraight(const double pos_x)
  {
    if (pos_x > 0.0)
    {
      ASSERT_GT(vehicle_model_ptr_->getX(), pos_x)
        << "Forward: It should be greater than " << pos_x;
    }
    else if (pos_x < 0.0)
    {
      ASSERT_LT(vehicle_model_ptr_->getX(), pos_x)
        << "Backward: It should be less than " << pos_x;
    }
  }

  void checkTurn(const double pos_y)
  {
    if (pos_y > 0.0)
    {
      ASSERT_GT(vehicle_model_ptr_->getY(), pos_y)
        << "Right turn: It should be greater than " << pos_y;
    }
    else if (pos_y < 0.0)
    {
      ASSERT_LT(vehicle_model_ptr_->getY(), pos_y)
        << "Left turn: It should be less than " << pos_y;
    }
  }

  void testGoStraightForward()
  {
    Eigen::VectorXd input(2);
    input << 3.0, 0.0;
    const double v_threshold = 0.5;
    const double x_threshold = 1.0;
    resetVehicleModel();
    updateNLoopWithConstantInput(input, 150);
    checkVel(v_threshold);
    checkStraight(x_threshold);
  }

  void testGoStraightBackward()
  {
    Eigen::VectorXd input(2);
    input << -3.0, 0.0;
    const double v_threshold = -0.5;
    const double x_threshold = -1.0;
    resetVehicleModel();
    updateNLoopWithConstantInput(input, 150);
    checkVel(v_threshold);
    checkStraight(x_threshold);
  }

  void testGoRightForward()
  {
    Eigen::VectorXd input(2);
    input << 3.0, 0.1;
    const double v_threshold = 0.5;
    const double y_threshold = 0.1;
    resetVehicleModel();
    updateNLoopWithConstantInput(input, 150);
    checkVel(v_threshold);
    checkTurn(y_threshold);
  }

  void testGoLeftForward()
  {
    Eigen::VectorXd input(2);
    input << 3.0, -0.1;
    const double v_threshold = 0.5;
    const double y_threshold = -0.1;
    resetVehicleModel();
    updateNLoopWithConstantInput(input, 150);
    checkVel(v_threshold);
    checkTurn(y_threshold);
  }

  void testAllMotion()
  {
    testGoStraightForward();
    testGoStraightBackward();
    testGoRightForward();
    testGoLeftForward();
  }
};

TEST_F(TestSuite, TestIdealTwist)
{
  vehicle_model_type_ = VehicleModelType::IDEAL_TWIST;
  testAllMotion();
}

TEST_F(TestSuite, TestIdealSteer)
{
  vehicle_model_type_ = VehicleModelType::IDEAL_STEER;
  testAllMotion();
}

TEST_F(TestSuite, TestDelayTwist)
{
  vehicle_model_type_ = VehicleModelType::DELAY_TWIST;
  testAllMotion();
}

TEST_F(TestSuite, TestDelaySteer)
{
  vehicle_model_type_ = VehicleModelType::DELAY_STEER;
  testAllMotion();
}

TEST_F(TestSuite, TestConstAccelTwist)
{
  vehicle_model_type_ = VehicleModelType::CONST_ACCEL_TWIST;
  testAllMotion();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
