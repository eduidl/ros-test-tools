#pragma once

#include <ros/node_handle.h>

#include <boost/function.hpp>

namespace ros_timer_factory
{
/**
 * @brief Modes of timer factory
 * - FactoryMode::Normal
 * - FactoryMode::Idle
 * - FactoryMode::ForceOnshot
 */
enum class FactoryMode {
  Normal,        //< Equal to `ros::createTimer` or so
  Idle,          //< Substitute given callback by empty function
  ForceOneshot,  //< Force to set oneshot true
};

namespace detail
{
template <typename TimerEvent>
void doNothing(const TimerEvent &)
{
}

template <typename TimerEvent, typename Duration>
auto createTimer(
  const ros::NodeHandle & nh, Duration period,
  const boost::function<void(const TimerEvent &)> & callback, bool oneshot, bool autostart);

template <>
inline auto createTimer(
  const ros::NodeHandle & nh, ros::Duration period, const ros::TimerCallback & callback,
  bool oneshot, bool autostart)
{
  return nh.createTimer(period, callback, oneshot, autostart);
}

template <>
inline auto createTimer(
  const ros::NodeHandle & nh, ros::WallDuration period, const ros::WallTimerCallback & callback,
  bool oneshot, bool autostart)
{
  return nh.createWallTimer(period, callback, oneshot, autostart);
}

template <>
inline auto createTimer(
  const ros::NodeHandle & nh, ros::WallDuration period, const ros::SteadyTimerCallback & callback,
  bool oneshot, bool autostart)
{
  return nh.createSteadyTimer(period, callback, oneshot, autostart);
}

template <typename TimerEvent, typename Duration>
class TimerFactory_
{
public:
  explicit TimerFactory_(ros::NodeHandle nh = ros::NodeHandle())
  : nh_{nh}, mode_{FactoryMode::Normal}
  {
  }

  TimerFactory_(const TimerFactory_ &) = default;
  TimerFactory_ & operator=(const TimerFactory_ &) = default;
  TimerFactory_(TimerFactory_ &&) = default;
  TimerFactory_ & operator=(TimerFactory_ &&) = default;

  virtual ~TimerFactory_() {}

  inline void reset(ros::NodeHandle & nh) { nh_ = nh; }

  void setMode(FactoryMode mode) { mode_ = mode; }

  template <typename T>
  auto create(
    Duration period, void (T::*fp)(const TimerEvent &) const, T * obj, bool oneshot = false,
    bool autostart = true) const
  {
    return this->create(period, boost::bind(fp, obj, _1), oneshot, autostart);
  }

  template <typename T>
  auto create(
    Duration period, void (T::*fp)(const TimerEvent &), T * obj, bool oneshot = false,
    bool autostart = true) const
  {
    return this->create(period, boost::bind(fp, obj, _1), oneshot, autostart);
  }

  auto create(
    Duration period, const boost::function<void(const TimerEvent &)> & callback,
    bool oneshot = false, bool autostart = true) const
  {
    if (mode_ == FactoryMode::Normal) {
      return createTimer(nh_, period, callback, oneshot, autostart);
    } else if (mode_ == FactoryMode::Idle) {
      return createTimer<TimerEvent, Duration>(
        nh_, period, &doNothing<TimerEvent>, oneshot, autostart);
    } else {
      return createTimer(nh_, period, callback, true, autostart);
    }
  }

private:
  ros::NodeHandle nh_;
  FactoryMode mode_;
};

}  // namespace detail

using TimerFactory = detail::TimerFactory_<ros::TimerEvent, ros::Duration>;
using WallTimerFactory = detail::TimerFactory_<ros::WallTimerEvent, ros::WallDuration>;
using SteadyTimerFactory = detail::TimerFactory_<ros::SteadyTimerEvent, ros::WallDuration>;

}  // namespace ros_timer_factory
