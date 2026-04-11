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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner::experimental
{

class SfmModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit SfmModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "sfm"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo info;
    info.predicted_objects = true;
    return info;
  }

private:
  SfmParameters params_;

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp,
    const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;
};

class SfmModulePlugin : public PluginWrapper<SfmModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // MANAGER_HPP_
