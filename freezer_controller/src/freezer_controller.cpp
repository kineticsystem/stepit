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

#include <freezer_controller/freezer_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace freezer_controller
{

FreezerController::FreezerController()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FreezerController::on_init()
{
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FreezerController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FreezerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < 18; ++i)
  {
    config.names.emplace_back("gpio/digital_output_cmd_" + std::to_string(i));
  }
  return config;
}

controller_interface::InterfaceConfiguration FreezerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FreezerController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    // io_pub_ = get_node()->create_publisher<ur_msgs::msg::IOStates>("~/io_states", rclcpp::SystemDefaultsQoS());
    set_io_srv_ = get_node()->create_service<freezer_msgs::srv::SetIO>(
        "~/set_io", [this](freezer_msgs::srv::SetIO::Request::SharedPtr req,
                           freezer_msgs::srv::SetIO::Response::SharedPtr resp) { this->setIO(req, resp); });
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool FreezerController::setIO([[maybe_unused]] freezer_msgs::srv::SetIO::Request::SharedPtr req,
                              [[maybe_unused]] freezer_msgs::srv::SetIO::Response::SharedPtr resp)
{
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FreezerController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FreezerController::update([[maybe_unused]] const rclcpp::Time& time,
                                                            [[maybe_unused]] const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

}  // namespace freezer_controller

PLUGINLIB_EXPORT_CLASS(freezer_controller::FreezerController, controller_interface::ControllerInterface)
