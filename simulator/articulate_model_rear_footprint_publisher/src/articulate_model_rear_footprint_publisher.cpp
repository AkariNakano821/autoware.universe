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

#include "articulate_model_rear_footprint_publisher/articulate_model_rear_footprint_publisher.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

namespace simulation
{
namespace articulate_model_rear_footprint_publisher
{

using geometry_msgs::msg::Point32;

ArticulateModelRearFootprintPublisher::ArticulateModelRearFootprintPublisher(
  const rclcpp::NodeOptions & options)
: Node("articulate_model_rear_footprint_publisher", options)
{
  sub_steering_report_ = create_subscription<SteeringReport>(
    "~/input/steering_report", 1,
    std::bind(
      &ArticulateModelRearFootprintPublisher::on_steering_report, this, std::placeholders::_1));
  pub_footprint_ = create_publisher<PolygonStamped>("~/output/polygon", 1);

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  const double rear_wheelbase_ratio = declare_parameter("rear_wheelbase_ratio", 1.0);

  const double width_2 = vehicle_info.wheel_tread_m * 0.5;
  width_left_ = width_2 + vehicle_info.left_overhang_m;
  width_right_ = width_2 + vehicle_info.right_overhang_m;
  length_ = vehicle_info.wheel_base_m * rear_wheelbase_ratio + vehicle_info.rear_overhang_m;
}

void ArticulateModelRearFootprintPublisher::on_steering_report(const SteeringReport::SharedPtr msg)
{
  PolygonStamped footprint;
  footprint.header.frame_id = "base_link";
  footprint.header.stamp = msg->stamp;
  footprint.polygon.points.resize(4);

  const auto s_steer = std::sin(msg->steering_tire_angle);
  const auto c_steer = std::cos(msg->steering_tire_angle);

  // front to rear
  Point32 front_to_rear;
  front_to_rear.x = -length_ * c_steer;
  front_to_rear.y = length_ * s_steer;
  front_to_rear.z = 0.0;

  // left front
  footprint.polygon.points[0].x = width_left_ * s_steer;
  footprint.polygon.points[0].y = width_left_ * c_steer;
  footprint.polygon.points[0].z = 0.0;

  // right front
  footprint.polygon.points[1].x = -width_right_ * s_steer;
  footprint.polygon.points[1].y = -width_right_ * c_steer;
  footprint.polygon.points[1].z = 0.0;

  // right rear
  footprint.polygon.points[2].x = footprint.polygon.points[1].x + front_to_rear.x;
  footprint.polygon.points[2].y = footprint.polygon.points[1].y + front_to_rear.y;
  footprint.polygon.points[2].z = 0.0;

  // left rear
  footprint.polygon.points[3].x = footprint.polygon.points[0].x + front_to_rear.x;
  footprint.polygon.points[3].y = footprint.polygon.points[0].y + front_to_rear.y;
  footprint.polygon.points[3].z = 0.0;

  pub_footprint_->publish(footprint);
}
}  // namespace articulate_model_rear_footprint_publisher
}  // namespace simulation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  simulation::articulate_model_rear_footprint_publisher::ArticulateModelRearFootprintPublisher)
