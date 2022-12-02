/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stepit_teleop/velocity_teleop_node.hpp>

namespace stepit_teleop
{
constexpr auto kLogger = "VelocityDriverNode";

VelocityTeleopNode::VelocityTeleopNode() : Node("velocity_teleop_node")
{
  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
  sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { callback(msg); });
}

void VelocityTeleopNode::callback([[maybe_unused]] const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto new_msg = std_msgs::msg::Float64MultiArray();
  const double values[] = { msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y };

  new_msg.data = { values[0], values[1], values[2], values[3], values[4] };
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "velocity command: {%f, %f, %f, %f, %f}", values[0], values[1], values[2],
               values[3], values[4]);
  pub_->publish(new_msg);
}
}  // namespace stepit_teleop
