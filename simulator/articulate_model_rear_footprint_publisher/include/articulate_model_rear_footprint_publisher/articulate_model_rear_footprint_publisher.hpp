// Copyright 2024 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARTICULATE_MODEL_REAR_FOOTPRINT_PUBLISHER__ARTICULATE_MODEL_REAR_FOOTPRINT_PUBLISHER_HPP_
#define ARTICULATE_MODEL_REAR_FOOTPRINT_PUBLISHER__ARTICULATE_MODEL_REAR_FOOTPRINT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace simulation
{
namespace articulate_model_rear_footprint_publisher
{

using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PolygonStamped;

class ArticulateModelRearFootprintPublisher : public rclcpp::Node
{
public:
  explicit ArticulateModelRearFootprintPublisher(const rclcpp::NodeOptions & options);

private:
  void on_steering_report(const SteeringReport::SharedPtr msg);
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_report_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr pub_footprint_;

  double width_left_;
  double width_right_;
  double length_;
};
}  // namespace articulate_model_rear_footprint_publisher
}  // namespace simulation

#endif  // ARTICULATE_MODEL_REAR_FOOTPRINT_PUBLISHER__ARTICULATE_MODEL_REAR_FOOTPRINT_PUBLISHER_HPP_