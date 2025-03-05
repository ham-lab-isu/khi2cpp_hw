// Copyright 2023 ros2_control Development Team
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

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

// Convenience aliases
using namespace std::chrono_literals;

// Global performance statistics structure
struct PerformanceStats {
  double last_loop_time = 0.0;
  double max_loop_time = 0.0;
  double total_loop_time = 0.0;
  unsigned loop_count = 0;
  std::mutex mtx;
};

// Our control loop class
class RealTimeControl : public rclcpp::Node
{
public:
  RealTimeControl()
  : Node("real_time_control"),
    stats_(std::make_shared<PerformanceStats>())
  {
    // Create a publisher for trajectory messages.
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/cx110l_controller/joint_trajectory", 10);

    // Create a diagnostics publisher.
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create a timer to publish diagnostics every second.
    diag_timer_ = this->create_wall_timer(1s, std::bind(&RealTimeControl::publishDiagnostics, this));

    // Optionally, get parameters (e.g. robot description, control period, etc.)
    this->declare_parameter<std::string>("robot_description", "");
    this->get_parameter("robot_description", robot_description_);

    // (Initialize trajectory, KDL chain, etc. here as needed)

    // Launch the control loop in a separate thread.
    control_thread_ = std::thread(&RealTimeControl::controlLoop, this);
  }

  ~RealTimeControl()
  {
    running_ = false;
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }

  private:
  void controlLoop()
  {
    // Use steady_clock for high-resolution timing.
    auto prev_time = std::chrono::steady_clock::now();
    // Set desired control loop period (e.g. 4ms).
    std::chrono::milliseconds period(4);
    while (rclcpp::ok() && running_) {
      auto loop_start = std::chrono::steady_clock::now();

      // --- Place hardware interface code here ---
      // For example:
      // - Read current joint states
      // - Update control computations (e.g. run a controller)
      // - Compute and publish a trajectory command
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.header.stamp = this->now();
      // (Fill in joint_names and points as needed)
      trajectory_pub_->publish(traj_msg);

      // --- End hardware interface code ---

      // Compute loop timing statistics.
      auto now_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> loop_duration = now_time - prev_time;
      prev_time = now_time;

      // Update performance stats.
      {
        std::lock_guard<std::mutex> lock(stats_->mtx);
        stats_->last_loop_time = loop_duration.count();
        stats_->max_loop_time = std::max(stats_->max_loop_time, stats_->last_loop_time);
        stats_->total_loop_time += stats_->last_loop_time;
        ++stats_->loop_count;
      }

      // Sleep until next period.
      std::this_thread::sleep_until(loop_start + period);
    }
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "RealTime Control Loop";
    status.hardware_id = "cx110l_controller";

    double avg_loop_time = 0.0;
    unsigned count = 0;
    double last = 0.0, max = 0.0;
    {
      std::lock_guard<std::mutex> lock(stats_->mtx);
      count = stats_->loop_count;
      if (count > 0) {
        avg_loop_time = stats_->total_loop_time / count;
      }
      last = stats_->last_loop_time;
      max = stats_->max_loop_time;
      // Reset statistics for next period if desired.
      stats_->total_loop_time = 0.0;
      stats_->loop_count = 0;
      stats_->max_loop_time = 0.0;
    }

    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
    status.values.resize(3);
    status.values[0].key = "Last Loop Time (s)";
    status.values[0].value = std::to_string(last);
    status.values[1].key = "Average Loop Time (s)";
    status.values[1].value = std::to_string(avg_loop_time);
    status.values[2].key = "Max Loop Time (s)";
    status.values[2].value = std::to_string(max);

    diag_array.status.push_back(status);
    diag_array.header.stamp = this->now();
    diag_pub_->publish(diag_array);
  }

  // Publishers and timers.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Control loop thread.
  std::thread control_thread_;
  std::atomic<bool> running_{true};

  // Performance statistics.
  std::shared_ptr<PerformanceStats> stats_;

  // Parameter(s)
  std::string robot_description_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Use a multi-threaded executor so that timers and our control loop thread can run concurrently.
  auto node = std::make_shared<RealTimeControl>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
