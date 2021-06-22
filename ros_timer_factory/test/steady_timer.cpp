#include <gtest/gtest.h>
#include <ros/ros.h>

#include "ros_timer_factory/ros_timer_factory.hpp"

namespace
{
class DummyNode
{
public:
  DummyNode() : count_{0} {}
  ~DummyNode() = default;

  void timerCallbackConst(const ros::SteadyTimerEvent &) const { count_++; }
  void timerCallback(const ros::SteadyTimerEvent &) { count_++; }

  mutable int count_;
};

class SteadyTimerFactoryTest : public ::testing::Test
{
protected:
  SteadyTimerFactoryTest() : spinner_{1} {}

  virtual void SetUp()
  {
    ros::init(ros::M_string(), "ros_timer_factory_test");
    spinner_ = ros::AsyncSpinner(1);
    spinner_.start();
    factory_ = ros_timer_factory::SteadyTimerFactory();
    node_ = DummyNode();
  }
  virtual void TearDown()
  {
    spinner_.stop();
    ros::shutdown();
  }

  ros::AsyncSpinner spinner_;
  ros_timer_factory::SteadyTimerFactory factory_;
  DummyNode node_;
};

void timerCallback(const ros::SteadyTimerEvent &) {}

}  // namespace

TEST_F(SteadyTimerFactoryTest, WithBareFunction)
{
  auto timer = factory_.create(ros::WallDuration(0.01), &timerCallback);
  EXPECT_EQ(timer.isValid(), true);
}

TEST_F(SteadyTimerFactoryTest, WithConstMemberFunction)
{
  auto timer =
    factory_.create(ros::WallDuration(0.01), &DummyNode::timerCallbackConst, &node_, true);
  ASSERT_EQ(timer.isValid(), true);

  ros::Duration(0.1).sleep();

  EXPECT_EQ(node_.count_, 1);
}

TEST_F(SteadyTimerFactoryTest, WithMemberFunction)
{
  auto timer = factory_.create(ros::WallDuration(0.01), &DummyNode::timerCallback, &node_, true);
  ASSERT_EQ(timer.isValid(), true);

  ros::Duration(0.1).sleep();

  EXPECT_EQ(node_.count_, 1);
}

TEST_F(SteadyTimerFactoryTest, WithBoostFunction)
{
  auto timer = factory_.create(
    ros::WallDuration(0.01), boost::bind(&DummyNode::timerCallback, &node_, _1), true);
  ASSERT_EQ(timer.isValid(), true);

  ros::Duration(0.1).sleep();

  EXPECT_EQ(node_.count_, 1);
}

TEST_F(SteadyTimerFactoryTest, NormalMode)
{
  factory_.setMode(ros_timer_factory::FactoryMode::Normal);
  auto timer = factory_.create(ros::WallDuration(0.01), &DummyNode::timerCallback, &node_);
  ASSERT_EQ(timer.isValid(), true);

  ros::Duration(0.1).sleep();

  EXPECT_GT(node_.count_, 5);
}

TEST_F(SteadyTimerFactoryTest, IdleMode)
{
  factory_.setMode(ros_timer_factory::FactoryMode::Idle);
  auto timer = factory_.create(ros::WallDuration(0.01), &DummyNode::timerCallback, &node_);
  ASSERT_EQ(timer.isValid(), true);

  ros::Duration(0.1).sleep();

  EXPECT_EQ(node_.count_, 0);
}

TEST_F(SteadyTimerFactoryTest, ForceOneshotMode)
{
  factory_.setMode(ros_timer_factory::FactoryMode::ForceOneshot);
  auto timer = factory_.create(ros::WallDuration(0.01), &DummyNode::timerCallback, &node_);
  ASSERT_EQ(timer.isValid(), true);

  ros::Duration(0.1).sleep();

  EXPECT_EQ(node_.count_, 1);
  EXPECT_FALSE(timer.hasPending());
}
