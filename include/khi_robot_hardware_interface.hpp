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

#ifndef KHI_ROBOT_HARDWARE_INTERFACE_
#define KHI_ROBOT_HARDWARE_INTERFACE_


#include <memory>
#include <vector>
#include <string>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// Remove the joint_limits_interface include since it’s not available by default in ROS2.
#include <khi_robot_client.h>

namespace khi_robot_control
{

class KhiRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KhiRobotHardwareInterface);

  KhiRobotHardwareInterface();
  ~KhiRobotHardwareInterface();

  // Lifecycle and initialization methods.
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Read and write methods now return hardware_interface::return_type.
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Additional robot-specific methods.
  bool open(const std::string & robot_name,
            const std::string & ip_address,
            const double & period,
            const bool in_simulation = false);
  bool activateRobot();
  bool hold();
  void deactivateRobot();
  void close();
  int updateState();
  int getStateTrigger();
  bool getPeriodDiff(double & diff);

private:
  // Storage for joint state and command data.
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;

  // Robot-specific members.
  khi_robot_control::KhiRobotData data;
  khi_robot_control::KhiRobotClient *client;
};

}  // namespace khi_robot_control

#endif  // KHI_ROBOT_HARDWARE_INTERFACE_