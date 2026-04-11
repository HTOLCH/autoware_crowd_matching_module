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

#include "scene.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

SfmModule::SfmModule(
  const int64_t module_id, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface,
  const SfmParameters & params)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  params_(params),
  prev_target_velocity_(params.desired_velocity),
  prev_time_(clock->now())
{
}

bool SfmModule::modifyPathVelocity(
  Trajectory & path,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  // Mode flags:
  //   "crowd_matching"          = crowd velocity + following (Config D, default)
  //   "classical"               = classical Helbing only (Config C)
  //   "crowd_only"              = crowd velocity, no following (Config E)
  //   "following_only"          = following controller only, no velocity model (Config F)
  //   "classical_following"     = classical Helbing + following (Config G)
  const bool use_classical = (params_.mode == "classical" || params_.mode == "classical_following");
  const bool use_crowd = (params_.mode == "crowd_matching" || params_.mode == "crowd_only" ||
                          params_.mode == "crowd_matching_pid");
  const bool use_following = (params_.mode == "crowd_matching" || params_.mode == "following_only" ||
                              params_.mode == "classical_following");
  const bool use_pid = (params_.mode == "crowd_matching_pid");
  RCLCPP_INFO_ONCE(logger_, "Crowd-Matching Module is executing (mode: %s)!", params_.mode.c_str());

  // Compute dt for smoothing
  auto now = clock_->now();
  double dt;
  if (first_call_) {
    dt = 0.1;
    first_call_ = false;
  } else {
    dt = (now - prev_time_).seconds();
  }
  prev_time_ = now;
  dt = std::clamp(dt, 0.01, 0.5);

  // Default target: speed limit (no crowd nearby)
  double raw_target = params_.desired_velocity;
  int co_flow_count = 0;
  double min_front_dist = params_.detection_radius;

  const auto & objects = planner_data.predicted_objects;
  const auto & ego_pose = planner_data.current_odometry->pose;

  // Ego forward direction from quaternion
  const auto & q = ego_pose.orientation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny, cosy);
  const double fwd_x = std::cos(yaw);
  const double fwd_y = std::sin(yaw);

  if (objects && !objects->objects.empty()) {
    using autoware_perception_msgs::msg::ObjectClassification;

    double co_flow_sum = 0.0;
    double total_force = 0.0;  // for classical mode
    const double ego_velocity = planner_data.current_velocity
                                   ? planner_data.current_velocity->twist.linear.x
                                   : 0.0;

    for (const auto & object : objects->objects) {
      if (object.classification.empty()) continue;
      if (object.classification[0].label != ObjectClassification::PEDESTRIAN) continue;

      const auto & pp = object.kinematics.initial_pose_with_covariance.pose.position;
      const auto & pt = object.kinematics.initial_twist_with_covariance.twist;

      const double dx = pp.x - ego_pose.position.x;
      const double dy = pp.y - ego_pose.position.y;
      const double dist = std::hypot(dx, dy);

      if (dist > params_.detection_radius) continue;
      if (dist < 1e-6) continue;

      // Forward and lateral distance relative to ego direction
      const double fwd_dist = dx * fwd_x + dy * fwd_y;
      const double lat_dist = std::abs(-dx * fwd_y + dy * fwd_x);

      if (use_classical) {
        // === Classical Helbing & Molnar 1995 repulsive force ===
        const double r_sum = params_.vehicle_half_width + params_.pedestrian_radius;
        const double safety_buffer = params_.safety_buffer_k * std::abs(ego_velocity);
        const double effective_distance = dist - safety_buffer;
        double force = params_.interaction_strength *
                       std::exp((r_sum - std::min(dist, effective_distance)) / params_.interaction_range);

        // Approach velocity amplification
        const double n_x = dx / dist;
        const double n_y = dy / dist;
        const double ped_approach_speed = -(pt.linear.x * n_x + pt.linear.y * n_y);
        if (ped_approach_speed > 0.0) {
          force *= (1.0 + ped_approach_speed);
        }

        total_force += std::min(force, params_.force_saturation);
      } else if (use_crowd) {
        // === Crowd-matching mode ===
        const double ped_fwd_speed = pt.linear.x * fwd_x + pt.linear.y * fwd_y;
        const bool is_co_flow = ped_fwd_speed > params_.co_flow_threshold;
        if (is_co_flow) {
          co_flow_sum += ped_fwd_speed;
          co_flow_count++;
        }
      }

      // Collision avoidance: ALL pedestrians in the bus's path (safety critical)
      const double lane_check = params_.vehicle_half_width + params_.pedestrian_radius + 0.5;
      if (fwd_dist > 0.0 && lat_dist < lane_check) {
        min_front_dist = std::min(min_front_dist, fwd_dist);
      }
    }

    if (use_classical) {
      // Classical: force-to-velocity mapping (linear)
      const double velocity_scale = std::max(0.0, 1.0 - total_force / params_.force_saturation);
      raw_target = params_.desired_velocity * velocity_scale;
    } else if (use_crowd) {
      // Crowd-matching: match co-flow pedestrian speed
      if (co_flow_count >= params_.min_crowd_size) {
        const double crowd_speed = co_flow_sum / co_flow_count;
        raw_target = std::min(raw_target, crowd_speed);
        last_crowd_speed_ = crowd_speed;
        time_since_crowd_ = 0.0;
      }
    }

  }

  // Hold crowd speed after crowd disappears — ramp up gradually over crowd_hold_time
  time_since_crowd_ += dt;
  if (time_since_crowd_ < params_.crowd_hold_time && last_crowd_speed_ > 0.0) {
    const double ramp = time_since_crowd_ / params_.crowd_hold_time;  // 0→1 over hold period
    const double held_ceiling = last_crowd_speed_ + ramp * (params_.desired_velocity - last_crowd_speed_);
    raw_target = std::min(raw_target, held_ceiling);
  }

  // Following distance controller — applied BEFORE smoothing so they work together
  // Targets zero velocity at min_clearance; Autoware backup stops below that
  if (use_following && min_front_dist < params_.braking_distance) {
    const double linear_scale = std::clamp(
      (min_front_dist - params_.min_clearance) / (params_.braking_distance - params_.min_clearance), 0.0, 1.0);
    // Quadratic: gentle at braking_distance, aggressive near min_clearance
    const double scale = linear_scale * linear_scale;
    raw_target = raw_target * scale;
  }

  // PID following distance controller — replaces proximity attenuation
  if (use_pid && min_front_dist < params_.detection_radius) {
    // Error: positive = too close, negative = far enough
    const double error = params_.pid_desired_distance - min_front_dist;
    pid_error_integral_ += error * dt;
    // Anti-windup: clamp integral
    pid_error_integral_ = std::clamp(pid_error_integral_, -5.0, 5.0);
    const double error_derivative = (error - pid_prev_error_) / dt;
    pid_prev_error_ = error;

    // PID output: velocity reduction (positive output = slow down)
    const double pid_output = params_.pid_kp * error
                            + params_.pid_ki * pid_error_integral_
                            + params_.pid_kd * error_derivative;
    raw_target = std::clamp(raw_target - pid_output, 0.0, params_.desired_velocity);
  }

  raw_target = std::max(raw_target, params_.min_velocity);

  // Smooth with relaxation time: exponential convergence toward raw_target
  const double alpha = 1.0 - std::exp(-dt / params_.relaxation_time);
  double target = prev_target_velocity_ + alpha * (raw_target - prev_target_velocity_);
  prev_target_velocity_ = target;

  // Apply to trajectory — cap each point's velocity at the target
  auto points = path.restore();
  for (auto & point : points) {
    if (point.point.longitudinal_velocity_mps > 1e-3) {
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps,
        static_cast<float>(target));
    }
  }

  const auto result = path.build(points);
  if (!result) {
    RCLCPP_WARN(logger_, "CM: Failed to rebuild trajectory");
    return false;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "CM: target=%.2f m/s, raw=%.2f, co_flow=%d, min_front=%.1f m",
    target, raw_target, co_flow_count, min_front_dist);

  // High-frequency log for plotting (parsed by analyze_results.py)
  {
    static FILE* sfm_log = fopen("/tmp/cm_target.csv", "w");
    static bool header_written = false;
    if (sfm_log && !header_written) {
      fprintf(sfm_log, "time,target,raw,co_flow,min_front\n");
      header_written = true;
    }
    if (sfm_log) {
      fprintf(sfm_log, "%.6f,%.4f,%.4f,%d,%.1f\n",
              clock_->now().seconds(), target, raw_target, co_flow_count, min_front_dist);
      fflush(sfm_log);
    }
  }

  return true;
}

visualization_msgs::msg::MarkerArray SfmModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray ma;
  return ma;
}

std::vector<autoware::motion_utils::VirtualWall> SfmModule::createVirtualWalls()
{
  std::vector<autoware::motion_utils::VirtualWall> vw;
  return vw;
}

}  // namespace autoware::behavior_velocity_planner::experimental
