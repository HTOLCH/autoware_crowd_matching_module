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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

struct SfmParameters
{
  // Algorithm mode: "crowd_matching" (default) or "classical" (Helbing repulsive force)
  std::string mode;
  // Target velocity when no crowd nearby (5 km/h campus speed limit)
  double desired_velocity;
  // Velocity convergence time constant (seconds)
  double relaxation_time;
  // Pedestrian body radius (meters)
  double pedestrian_radius;
  // Bus half-width for lateral path check (meters)
  double vehicle_half_width;
  // Absolute velocity floor (m/s)
  double min_velocity;
  // Max distance to consider pedestrians (meters)
  double detection_radius;
  // Forward distance at which collision avoidance ramp begins (meters)
  double braking_distance;
  // Minimum clearance — module targets zero velocity at this distance (meters)
  double min_clearance;
  // Min pedestrian forward speed to count as co-flow (m/s)
  double co_flow_threshold;
  // Minimum number of co-flow pedestrians to trigger crowd matching
  int min_crowd_size;
  // How long to hold crowd speed after crowd disappears (seconds)
  double crowd_hold_time;
  // Classical parameters (Helbing & Molnar 1995)
  double interaction_strength;  // A parameter
  double interaction_range;     // B parameter (meters)
  double safety_buffer_k;       // speed-proportional buffer multiplier
  double force_saturation;      // force magnitude at which velocity -> 0
  // PID following controller parameters
  double pid_desired_distance;  // target following gap (meters)
  double pid_kp;                // proportional gain
  double pid_ki;                // integral gain
  double pid_kd;                // derivative gain
};

class SfmModule : public SceneModuleInterface
{
public:
  SfmModule(
    const int64_t module_id, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface,
    const SfmParameters & params);

  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

  std::vector<autoware::motion_utils::VirtualWall> createVirtualWalls() override;

private:
  SfmParameters params_;
  double prev_target_velocity_;
  double smoothed_crowd_speed_{0.0};
  double last_crowd_speed_{0.0};
  double time_since_crowd_{100.0};  // large initial = no crowd history
  bool first_call_{true};
  rclcpp::Time prev_time_;
  // PID state
  double pid_error_integral_{0.0};
  double pid_prev_error_{0.0};
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // SCENE_HPP_
