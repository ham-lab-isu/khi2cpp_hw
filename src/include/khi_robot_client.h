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
 *     copyright notice, this list of the conditions and the following
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

#ifndef KHI_ROBOT_CLIENT_HPP
#define KHI_ROBOT_CLIENT_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

// Replace with your actual driver header as needed.
// For example: #include "khi_robot_krnx_driver.h"
#include "khi_robot_driver.h"

namespace khi_robot_control
{

class KhiRobotClient
{
public:
  KhiRobotClient();
  ~KhiRobotClient();

  /**
   * @brief Open a connection to the robot
   * @param ip IP address or hostname of the controller
   * @param period Control cycle period in seconds
   * @param data Reference to a KhiRobotData structure for input/output
   * @param in_simulation True if running in a simulation environment
   * @return True if successful, false otherwise
   */
  bool open(const std::string &ip, 
            const double &period, 
            KhiRobotData &data, 
            bool in_simulation = false);

  /**
   * @brief Activate the robot (e.g., servo on)
   * @param data Reference to a KhiRobotData structure for input/output
   * @return True if successful, false otherwise
   */
  bool activate(KhiRobotData &data);

  /**
   * @brief Hold the robot (e.g., servo pause)
   * @param data The current robot data
   * @return True if successful, false otherwise
   */
  bool hold(const KhiRobotData &data);

  /**
   * @brief Deactivate the robot (e.g., servo off)
   * @param data The current robot data
   */
  void deactivate(const KhiRobotData &data);

  /**
   * @brief Close the driver connection
   */
  void close();

  /**
   * @brief Write control commands to the robot
   * @param data The control data to be sent
   */
  void write(const KhiRobotData &data);

  /**
   * @brief Read updated state from the robot
   * @param data The structure to store the read state
   */
  void read(KhiRobotData &data);

  /**
   * @brief Update internal state from driver
   * @param data The current robot data
   * @return Driver state constant (e.g., OK, NOT_REGISTERED, etc.)
   */
  int updateState(const KhiRobotData &data);

  /**
   * @brief Retrieve the state trigger from the driver
   * @return Trigger enum or integer describing the driver’s state
   */
  int getStateTrigger();

  /**
   * @brief Query the driver for a difference between expected and actual period
   * @param diff Output argument for storing the difference
   * @return True if successful, false otherwise
   */
  bool getPeriodDiff(double &diff);

  /**
   * @brief Start the command service in a separate thread
   */
  void startCommandService();

private:
  int cont_no;                ///< Controller number or index
  KhiRobotDriver *driver;     ///< Pointer to the underlying driver interface
};

}  // namespace khi_robot_control

#endif  // KHI_ROBOT_CLIENT_HPP
