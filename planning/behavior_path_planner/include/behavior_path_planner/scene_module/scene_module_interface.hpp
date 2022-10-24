// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <behavior_path_planner/steering_factor_interface.hpp>
#include <behavior_path_planner/turn_signal_decider.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>
#include <rtc_interface/rtc_interface.hpp>

#include <autoware_adapi_v1_msgs/msg/steering_factor_array.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <behaviortree_cpp_v3/basic_types.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_adapi_v1_msgs::msg::SteeringFactor;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using rtc_interface::RTCInterface;
using steering_factor_interface::SteeringFactorInterface;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;
using PlanResult = PathWithLaneId::SharedPtr;

struct BehaviorModuleOutput
{
  BehaviorModuleOutput() = default;

  // path planed by module
  PlanResult path{};

  // path candidate planed by module
  PlanResult path_candidate{};

  TurnSignalInfo turn_signal_info{};
};

struct CandidateOutput
{
  CandidateOutput() = default;
  explicit CandidateOutput(const PathWithLaneId & path) : path_candidate{path} {}
  PathWithLaneId path_candidate{};
  double lateral_shift{0.0};
  double start_distance_to_path_change{std::numeric_limits<double>::lowest()};
  double finish_distance_to_path_change{std::numeric_limits<double>::lowest()};
};

class SceneModuleInterface
{
public:
  SceneModuleInterface(const std::string & name, rclcpp::Node & node)
  : name_{name},
    logger_{node.get_logger().get_child(name)},
    clock_{node.get_clock()},
    uuid_(generateUUID()),
    is_waiting_approval_{false},
    current_state_{BT::NodeStatus::IDLE}
  {
    std::string module_ns;
    module_ns.resize(name.size());
    std::transform(name.begin(), name.end(), module_ns.begin(), tolower);

    const auto ns = std::string("~/debug/") + module_ns;
    pub_debug_marker_ = node.create_publisher<MarkerArray>(ns, 20);
  }


  std::string name() const { return name_; }

  rclcpp::Logger getLogger() const { return logger_; }


private:
  std::string name_;
  rclcpp::Logger logger_;

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;
  mutable AvoidanceDebugMsgArray::SharedPtr debug_avoidance_msg_array_ptr_{};
  mutable MarkerArray debug_marker_;

  std::shared_ptr<RTCInterface> rtc_interface_ptr_;
  std::unique_ptr<SteeringFactorInterface> steering_factor_interface_ptr_;
  UUID uuid_;

};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
