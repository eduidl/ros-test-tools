#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(AAA, FAIL) { ASSERT_TRUE(false); }

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ros_timer_wrap_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
