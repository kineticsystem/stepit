// Copyright 2023 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <limits>
#include <vector>
#include <string>
#include <cmath>

#include <freezer_driver/freezer_hardware.hpp>
#include <freezer_driver/msgs/acknowledge_response.hpp>
#include <freezer_driver/msgs/status_response.hpp>

#include <freezer_driver/default_driver_factory.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace freezer_driver
{
const auto kLogger = rclcpp::get_logger("freezer_hardware");

FreezerHardware::FreezerHardware()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

// This constructor is use for testing only.
FreezerHardware::FreezerHardware(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
}

hardware_interface::CallbackReturn FreezerHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(kLogger, "on_init");
  try
  {
    // Store hardware info for later use.
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    driver_ = driver_factory_->create(info);
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    return CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FreezerHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    if (!driver_->connect())
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to the device.");
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot connect to the device: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FreezerHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");
  return {};
}

std::vector<hardware_interface::CommandInterface> FreezerHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");
  return {};
}

hardware_interface::CallbackReturn
FreezerHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FreezerHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FreezerHardware::read(const rclcpp::Time&,
                                                      [[maybe_unused]] const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

// How to set velocity commands using the command line:
// ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5,0.1]"
hardware_interface::return_type FreezerHardware::write(const rclcpp::Time&,
                                                       [[maybe_unused]] const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

}  // namespace freezer_driver

PLUGINLIB_EXPORT_CLASS(freezer_driver::FreezerHardware, hardware_interface::SystemInterface)
