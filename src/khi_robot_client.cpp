/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Kawasaki Heavy Industries, LTD.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>

// Replace this with your actual service message type.
// For example: #include <khi_robot_interfaces/srv/command.hpp>
// Or #include <your_pkg/srv/command.hpp>
#include <khi_robot_msgs/srv/khi_robot_cmd.hpp>

#include <khi_robot_client.h>
#include <khi_robot_krnx_driver.h>

namespace khi_robot_control
{

/**
 * @brief Helper function to spin a Node providing the "khi_robot_command_service".
 *
 * The original ROS1 code used a ros::AsyncSpinner plus a blocking ros::waitForShutdown().
 * In ROS2, we'll create a separate node, attach a service to it, and spin in a MultiThreadedExecutor.
 */
void KhiCommandService(KhiRobotDriver *driver)
{
  if (driver == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("KhiCommandService"),
                 "Driver pointer is null. Cannot start service.");
    return;
  }

  // Create a dedicated node for the command service
  auto node = rclcpp::Node::make_shared("khi_robot_command_service_node");
  RCLCPP_INFO(node->get_logger(), "Starting KhiCommandService node");

  // Create the service using your actual service type: KhiRobotCmd
  auto srv = node->create_service<khi_robot_msgs::srv::KhiRobotCmd>(
    "khi_robot_command_service",
    [driver](const std::shared_ptr<khi_robot_msgs::srv::KhiRobotCmd::Request> request,
             std::shared_ptr<khi_robot_msgs::srv::KhiRobotCmd::Response> response)
    {
      // The driver’s commandHandler expects references, so dereference
      bool success = driver->commandHandler(*request, *response);

      // You might populate extra fields in the response if needed
      // e.g. response->error_code = success ? 0 : 1;
      // or response->message = success ? "Succeeded" : "Failed";

      // Log the request or result if desired
      RCLCPP_INFO(rclcpp::get_logger("KhiCommandService"),
                  "Service call processed. Success: %s", success ? "true" : "false");
    }
  );

  // Use a MultiThreadedExecutor if you want concurrency. A single-threaded
  // executor would also work if you do not need parallel callbacks.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
}

//---------------------------------------------------------------------------------------
// Implementation of KhiRobotClient in ROS2
//---------------------------------------------------------------------------------------

bool KhiRobotClient::open(const std::string &ip,
                          const double &period,
                          KhiRobotData &data,
                          const bool in_simulation)
{
  cont_no = 0;

  // Create and initialize the driver (Replace with your actual driver class and logic)
  driver = new KhiRobotKrnxDriver();
  if (!driver->initialize(cont_no, period, data, in_simulation)) {
    return false;
  }

  // Open the connection
  if (!driver->open(cont_no, ip, data)) {
    return false;
  }

  // Start the command service in a separate thread
  startCommandService();

  return true;
}

bool KhiRobotClient::activate(KhiRobotData &data)
{
  if (driver == nullptr) {
    return false;
  }
  return driver->activate(cont_no, data);
}

bool KhiRobotClient::hold(const KhiRobotData &data)
{
  if (driver == nullptr) {
    return false;
  }
  return driver->hold(cont_no, data);
}

void KhiRobotClient::deactivate(const KhiRobotData &data)
{
  if (driver == nullptr) {
    return;
  }
  driver->deactivate(cont_no, data);
}

void KhiRobotClient::close()
{
  if (driver == nullptr) {
    return;
  }
  driver->close(cont_no);
  delete driver;
  driver = nullptr;
}

void KhiRobotClient::write(const KhiRobotData &data)
{
  if (driver == nullptr) {
    return;
  }
  driver->writeData(cont_no, data);
}

void KhiRobotClient::read(KhiRobotData &data)
{
  if (driver == nullptr) {
    return;
  }
  driver->readData(cont_no, data);
}

int KhiRobotClient::updateState(const KhiRobotData &data)
{
  if (driver == nullptr) {
    return NOT_REGISTERED;
  }
  return driver->updateState(cont_no, data);
}

int KhiRobotClient::getStateTrigger()
{
  if (driver == nullptr) {
    return NONE;
  }
  return driver->getStateTrigger(cont_no);
}

bool KhiRobotClient::getPeriodDiff(double &diff)
{
  if (driver == nullptr) {
    return false;
  }
  return driver->getPeriodDiff(cont_no, diff);
}

void KhiRobotClient::startCommandService()
{
  if (driver == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("KhiRobotClient"), 
                 "Driver pointer is null. Cannot start command service.");
    return;
  }

  // Launch the service node in a separate thread
  std::thread thread_srv(KhiCommandService, driver);
  thread_srv.detach();
}

}  // end of namespace khi_robot_control
