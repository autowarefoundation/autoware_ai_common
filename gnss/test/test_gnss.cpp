#include <ros/ros.h>
#include <gtest/gtest.h>

#include "gnss/geo_pos_conv.hpp"

TEST(TestSuite, llhNmeaDegreesTest) {

  geo_pos_conv geo_;

  geo_.set_llh_nmea_degrees(0.000000,0.000000,10.124025);
  ASSERT_FLOAT_EQ(0.000000, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(0.000000, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(1.000000,1.000000,10.124025);
  ASSERT_FLOAT_EQ(1842.720386, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(1855.139262, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(-1.000000,-1.000000,10.124025);
  ASSERT_FLOAT_EQ(-1842.720386, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(-1855.139262, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(10.000000,10.000000,10.124025);
  ASSERT_FLOAT_EQ(18427.282074, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(18551.341517, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(-10.000000,-10.000000,10.124025);
  ASSERT_FLOAT_EQ(-18427.282074, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(-18551.341517, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(100.000000,100.000000,10.124025);
  ASSERT_FLOAT_EQ(110580.283099, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(111297.204780, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());

  geo_.set_llh_nmea_degrees(-100.000000,-100.000000,10.124025);
  ASSERT_FLOAT_EQ(-110580.283099, geo_.geo_pos_conv::x());
  ASSERT_FLOAT_EQ(-111297.204780, geo_.geo_pos_conv::y());
  ASSERT_FLOAT_EQ(10.124025, geo_.geo_pos_conv::z());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_gnss");
  return RUN_ALL_TESTS();
}
