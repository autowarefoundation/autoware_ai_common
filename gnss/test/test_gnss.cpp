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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "gnss/geo_pos_conv.hpp"

TEST(TestSuite, llhNmeaDegreesTest)
{
  geo_pos_conv geo_;

  geo_.set_llh_nmea_degrees(0.000000, 0.000000, 10.124025);
  ASSERT_FLOAT_EQ(0.000000, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(0.000000, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(1.000000, 1.000000, 10.124025);
  ASSERT_FLOAT_EQ(1842.720386, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(1855.139262, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(-1.000000, -1.000000, 10.124025);
  ASSERT_FLOAT_EQ(-1842.720386, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(-1855.139262, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(10.000000, 10.000000, 10.124025);
  ASSERT_FLOAT_EQ(18427.282074, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(18551.341517, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(-10.000000, -10.000000, 10.124025);
  ASSERT_FLOAT_EQ(-18427.282074, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(-18551.341517, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(100.000000, 100.000000, 10.124025);
  ASSERT_FLOAT_EQ(110580.283099, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(111297.204780, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(-100.000000, -100.000000, 10.124025);
  ASSERT_FLOAT_EQ(-110580.283099, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(-111297.204780, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_gnss");
  return RUN_ALL_TESTS();
}
