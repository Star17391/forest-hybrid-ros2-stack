#ifndef FOREST_SENSORS_CPP__QOS_PROFILES_HPP_
#define FOREST_SENSORS_CPP__QOS_PROFILES_HPP_

#include "rclcpp/qos.hpp"

namespace forest_sensors_cpp
{

inline rclcpp::QoS best_effort_sensor_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

}  // namespace forest_sensors_cpp

#endif  // FOREST_SENSORS_CPP__QOS_PROFILES_HPP_
