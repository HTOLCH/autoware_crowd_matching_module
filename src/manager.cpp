// Copyright 2026 Harry, UWA
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

#include "manager.hpp"

#include <autoware_utils_rclcpp/parameter.hpp>

#include <memory>
#include <string>

namespace autoware::behavior_velocity_planner::experimental
{
using autoware_utils_rclcpp::get_or_declare_parameter;

SfmModuleManager::SfmModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  std::string ns(SfmModuleManager::getModuleName());
  params_.mode = get_or_declare_parameter<std::string>(node, ns + ".mode");
  params_.desired_velocity = get_or_declare_parameter<double>(node, ns + ".desired_velocity");
  params_.relaxation_time = get_or_declare_parameter<double>(node, ns + ".relaxation_time");
  params_.pedestrian_radius = get_or_declare_parameter<double>(node, ns + ".pedestrian_radius");
  params_.vehicle_half_width = get_or_declare_parameter<double>(node, ns + ".vehicle_half_width");
  params_.min_velocity = get_or_declare_parameter<double>(node, ns + ".min_velocity");
  params_.detection_radius = get_or_declare_parameter<double>(node, ns + ".detection_radius");
  params_.braking_distance = get_or_declare_parameter<double>(node, ns + ".braking_distance");
  params_.min_clearance = get_or_declare_parameter<double>(node, ns + ".min_clearance");
  params_.co_flow_threshold = get_or_declare_parameter<double>(node, ns + ".co_flow_threshold");
  params_.min_crowd_size = get_or_declare_parameter<int>(node, ns + ".min_crowd_size");
  params_.crowd_hold_time = get_or_declare_parameter<double>(node, ns + ".crowd_hold_time");
  // Classical parameters (only used when mode=classical)
  params_.interaction_strength = get_or_declare_parameter<double>(node, ns + ".interaction_strength");
  params_.interaction_range = get_or_declare_parameter<double>(node, ns + ".interaction_range");
  params_.safety_buffer_k = get_or_declare_parameter<double>(node, ns + ".safety_buffer_k");
  params_.force_saturation = get_or_declare_parameter<double>(node, ns + ".force_saturation");
  // PID following controller parameters
  params_.pid_desired_distance = get_or_declare_parameter<double>(node, ns + ".pid_desired_distance");
  params_.pid_kp = get_or_declare_parameter<double>(node, ns + ".pid_kp");
  params_.pid_ki = get_or_declare_parameter<double>(node, ns + ".pid_ki");
  params_.pid_kd = get_or_declare_parameter<double>(node, ns + ".pid_kd");
}

void SfmModuleManager::launchNewModules(
  [[maybe_unused]] const Trajectory & path,
  [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(
      std::make_shared<SfmModule>(
        module_id, logger_.get_child(getModuleName()), clock_, time_keeper_,
        planning_factor_interface_, params_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SfmModuleManager::getModuleExpiredFunction(
  [[maybe_unused]] const Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return []([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) -> bool {
    return false;
  };
}

}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::SfmModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
