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

#include <algorithm>
#include <limits>
#include <vector>
#include <string>
#include <cmath>

#include <stepit_hardware/stepit_hardware.hpp>
#include <stepit_driver/msgs/velocity_command.hpp>
#include <stepit_driver/msgs/position_command.hpp>
#include <stepit_driver/msgs/acknowledge_response.hpp>
#include <stepit_driver/msgs/info_response.hpp>
#include <stepit_driver/msgs/status_response.hpp>

#include <stepit_driver/default_driver_factory.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace stepit_driver
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// Tolerance applied when comparing a declared value against a reported limit.
constexpr double kLimitTolerance = 1.001;

// Upper bound on the number of joints, set by the bitmask motor_states_are_valid()
// uses to track which of them a status response has already reported.
constexpr std::size_t kMaxJoints = 32;

const auto kLogger = rclcpp::get_logger("stepit_hardware");

StepitHardware::StepitHardware()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

// This constructor is use for testing only.
StepitHardware::StepitHardware(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
}

hardware_interface::CallbackReturn
StepitHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  RCLCPP_DEBUG(kLogger, "on_init");
  try
  {
    // Store hardware info for later use.
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Initialize all joints. Reset the derived state too: on a second call
    // stale entries would make every configured id look like a duplicate.
    const std::size_t num_joints = info_.joints.size();
    // Joint holds atomics, so the vector is rebuilt rather than assigned.
    joints_ = std::vector<Joint>(num_joints);
    joint_index_by_id_.clear();

    // motor_states_are_valid() tracks the joints it has seen in a bitmask, so
    // there cannot be more joints than the mask has bits.
    if (num_joints > kMaxJoints)
    {
      RCLCPP_ERROR(kLogger, "%zu joints are declared but at most %zu are supported.", num_joints, kMaxJoints);
      return CallbackReturn::ERROR;
    }

    for (uint i = 0; i < num_joints; i++)
    {
      joints_[i].id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters.at("id")));
      joints_[i].acceleration = std::stod(info_.joints[i].parameters.at("acceleration"));
      joints_[i].max_velocity = std::stod(info_.joints[i].parameters.at("max_velocity"));
      joints_[i].state.position = kNaN;
      joints_[i].state.velocity = kNaN;
      joints_[i].command.position = kNaN;
      joints_[i].command.velocity = kNaN;
      RCLCPP_INFO(kLogger, "joint_id %d: %d", i, joints_[i].id);

      // The controller reports one state per physical motor, indexed 0..N-1,
      // so a configured id must fall in that range. Reject it here with a
      // clear message instead of failing later at configure time.
      if (static_cast<std::size_t>(joints_[i].id) >= num_joints)
      {
        RCLCPP_ERROR(kLogger, "joint %d has motor id %d which is out of range [0, %d] for %d joints.",
                     static_cast<int>(i), static_cast<int>(joints_[i].id), static_cast<int>(num_joints - 1),
                     static_cast<int>(num_joints));
        return CallbackReturn::ERROR;
      }

      // Map the motor id to this joint's index so read() can route a reported
      // state to the right joint. Reject a duplicate id: it would make the
      // mapping ambiguous and is a configuration error.
      auto [it, inserted] = joint_index_by_id_.emplace(joints_[i].id, i);
      if (!inserted)
      {
        RCLCPP_ERROR(kLogger, "motor id %d is used by both joint %d and joint %d; ids must be unique.", joints_[i].id,
                     static_cast<int>(it->second), static_cast<int>(i));
        return CallbackReturn::ERROR;
      }
    }

    driver_ = driver_factory_->create(info_);
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    return CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StepitHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake: the driver verifies that the device
    // on the other end identifies itself as a StepIt controller.
    if (!driver_->connect())
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to the StepIt controller.");
      return CallbackReturn::FAILURE;
    }

    // Ask the controller what its motors tolerate and check the values the
    // URDF declares against them. The controller is the authority: it knows
    // the board and the mechanics, the description does not. Validating here
    // turns what would be an opaque rejection from the firmware into a
    // message naming the joint and the limit it exceeds.
    const InfoResponse info = driver_->get_info(rclcpp::Time{});
    if (info.status() != Response::Status::Success)
    {
      RCLCPP_ERROR(kLogger, "The StepIt controller did not report its limits.");
      return CallbackReturn::FAILURE;
    }
    if (!joints_are_within_limits(info.limits()))
    {
      return CallbackReturn::FAILURE;
    }

    // Send configuration parameters to the hardware.
    std::vector<ConfigParam> params;
    for (const auto& joint : joints_)
    {
      params.emplace_back(ConfigParam{ joint.id, joint.acceleration, joint.max_velocity });
    }
    const AcknowledgeResponse response = driver_->configure(ConfigCommand{ params });
    if (response.status() == Response::Status::Failure)
    {
      return CallbackReturn::FAILURE;
    }

    // Verify that the controller reports exactly the motors configured above,
    // so that a mismatch is reported here rather than on every read cycle.
    // This has to happen after configure: the fake driver only creates its
    // motors once it has received the configuration.
    const StatusResponse status = driver_->get_status(rclcpp::Time{});
    if (status.status() != Response::Status::Success)
    {
      RCLCPP_ERROR(kLogger, "The StepIt controller did not report its motors status.");
      return CallbackReturn::FAILURE;
    }
    // A matching motor count does not prove the id mapping: the controller
    // could report an out-of-range or a duplicated id and still match the
    // count. Verify the full set so a bad mapping fails here, not on every
    // read cycle.
    if (!motor_states_are_valid(status.motor_states()))
    {
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    set_lifecycle_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                                hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> StepitHardware::on_export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "on_export_state_interfaces");
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> StepitHardware::on_export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "on_export_command_interfaces");
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
StepitHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
StepitHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");
  try
  {
    // Bring every motor to rest. The firmware keeps executing the last goal
    // it received, and its watchdog does not fire while anything, the status
    // queries included, keeps talking to it. A zero velocity decelerates at
    // the configured acceleration rather than cutting the motion dead.
    std::vector<VelocityGoal> stops;
    for (auto& joint : joints_)
    {
      stops.emplace_back(VelocityGoal{ joint.id, 0.0 });
      joint.stop_pending = false;
    }
    const AcknowledgeResponse response = driver_->set_velocity(rclcpp::Time{}, VelocityCommand{ stops });
    if (response.status() != Response::Status::Success)
    {
      RCLCPP_ERROR(kLogger, "The StepIt controller rejected the command to stop the motors.");
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(kLogger, "Cannot stop the motors: %s", ex.what());
    return CallbackReturn::ERROR;
  }
}

bool StepitHardware::joints_are_within_limits(const std::vector<MotorLimits>& limits) const
{
  for (const auto& joint : joints_)
  {
    const auto it = std::find_if(limits.cbegin(), limits.cend(),
                                 [&joint](const MotorLimits& limit) { return limit.id() == joint.id; });
    if (it == limits.cend())
    {
      RCLCPP_ERROR(kLogger, "The StepIt controller reported no limits for motor %d.", joint.id);
      return false;
    }

    // A tolerance is applied because a description normally states the limit
    // itself, and the round trip through the controller rounds it.
    if (joint.acceleration > it->max_acceleration() * kLimitTolerance)
    {
      RCLCPP_ERROR(kLogger, "Motor %d: acceleration %f rad/s^2 exceeds the controller limit of %f rad/s^2.", joint.id,
                   joint.acceleration, it->max_acceleration());
      return false;
    }
    if (joint.max_velocity > it->max_velocity() * kLimitTolerance)
    {
      RCLCPP_ERROR(kLogger, "Motor %d: max velocity %f rad/s exceeds the controller limit of %f rad/s.", joint.id,
                   joint.max_velocity, it->max_velocity());
      return false;
    }
  }
  return true;
}

bool StepitHardware::motor_states_are_valid(const std::vector<MotorState>& motor_states) const
{
  if (motor_states.size() != joints_.size())
  {
    RCLCPP_ERROR(kLogger, "status reports %zu motors but %zu joints are configured.", motor_states.size(),
                 joints_.size());
    return false;
  }

  // One bit per joint, so this runs without allocating on the read cycle.
  // on_init() guarantees there are no more joints than bits.
  uint32_t seen = 0;
  for (const auto& state : motor_states)
  {
    auto it = joint_index_by_id_.find(state.id());
    if (it == joint_index_by_id_.end())
    {
      RCLCPP_ERROR(kLogger, "status reports motor id %d which is not a configured joint.", state.id());
      return false;
    }
    const uint32_t bit = uint32_t{ 1 } << it->second;
    if (seen & bit)
    {
      RCLCPP_ERROR(kLogger, "status reports motor id %d more than once.", state.id());
      return false;
    }
    seen |= bit;
  }
  return true;
}

hardware_interface::return_type StepitHardware::read(const rclcpp::Time& time,
                                                     [[maybe_unused]] const rclcpp::Duration& period)
{
  try
  {
    StatusResponse response = driver_->get_status(time);

    auto motor_states = response.motor_states();
    if (!motor_states_are_valid(motor_states))
    {
      return hardware_interface::return_type::ERROR;
    }

    for (const auto& state : motor_states)
    {
      // motor_states_are_valid() guarantees every reported id maps to a
      // distinct configured joint, so this lookup cannot miss or collide.
      const std::size_t index = joint_index_by_id_.at(state.id());
      joints_[index].state.position = state.position();
      joints_[index].state.velocity = state.velocity();
    }

    return hardware_interface::return_type::OK;
  }
  catch (const std::exception& ex)
  {
    set_lifecycle_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                                hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return hardware_interface::return_type::ERROR;
  }
}

// How to set velocity commands using the command line:
// ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5,0.1]"
hardware_interface::return_type StepitHardware::write(const rclcpp::Time& time,
                                                      [[maybe_unused]] const rclcpp::Duration& period)
{
  try
  {
    // A joint is only ever commanded below if its command interface is both
    // currently claimed by an active controller (velocity_claimed/
    // position_claimed, tracked by perform_command_mode_switch) AND holds a
    // real value. Requiring both, rather than just the claim, avoids ever
    // forwarding a transient NaN (e.g. the moment a controller activates but
    // hasn't written its first real command yet) down to the driver, since
    // the fake motor's kinematics latch onto whatever value they last saw.
    auto velocity_commanded = [](const Joint& joint) {
      return joint.velocity_claimed && !std::isnan(joint.command.velocity);
    };
    auto position_commanded = [](const Joint& joint) {
      return joint.position_claimed && !std::isnan(joint.command.position);
    };

    // Velocity commands take precedence: position goals are only sent when no
    // joint is velocity controlled.
    const bool velocity_mode = std::any_of(joints_.cbegin(), joints_.cend(), velocity_commanded);

    std::vector<VelocityGoal> velocities;
    std::vector<PositionGoal> positions;
    for (auto& joint : joints_)
    {
      // Take the flag before reading the claims: a release clears the claim
      // before raising the flag, so seeing the flag guarantees seeing the
      // release. A release that lands later stays pending for the next cycle.
      const bool stop_pending = joint.stop_pending.exchange(false);
      if (velocity_commanded(joint))
      {
        velocities.emplace_back(VelocityGoal{ joint.id, joint.command.velocity });
      }
      else if (!velocity_mode && position_commanded(joint))
      {
        positions.emplace_back(PositionGoal{ joint.id, joint.command.position });
      }
      else if (stop_pending)
      {
        // A controller released this joint and nothing commands it now. The
        // firmware would otherwise carry on with the last goal, so bring the
        // motor to rest.
        velocities.emplace_back(VelocityGoal{ joint.id, 0.0 });
      }
    }

    if (!velocities.empty())
    {
      const AcknowledgeResponse response = driver_->set_velocity(time, VelocityCommand{ velocities });
      if (response.status() != Response::Status::Success)
      {
        // The controller refused the command. It stops the motors itself, but
        // report the failure so that ros2_control deactivates the component
        // instead of the loop carrying on as though the goal had been taken.
        RCLCPP_ERROR(kLogger, "The StepIt controller rejected a velocity command.");
        return hardware_interface::return_type::ERROR;
      }
    }
    if (!positions.empty())
    {
      const AcknowledgeResponse response = driver_->set_position(time, PositionCommand{ positions });
      if (response.status() != Response::Status::Success)
      {
        RCLCPP_ERROR(kLogger, "The StepIt controller rejected a position command.");
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }
  catch (const std::exception& ex)
  {
    set_lifecycle_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                                hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type
StepitHardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                            const std::vector<std::string>& stop_interfaces)
{
  auto set_claim = [this](const std::vector<std::string>& interface_names, bool claimed) {
    for (const auto& interface_name : interface_names)
    {
      auto slash = interface_name.find('/');
      std::string joint_name = interface_name.substr(0, slash);
      std::string interface_type = interface_name.substr(slash + 1);

      for (std::size_t i = 0; i < info_.joints.size(); ++i)
      {
        if (info_.joints[i].name == joint_name)
        {
          // A newly claimed interface starts with no command. Controllers do
          // not clear it on activation: a forward controller writes nothing
          // until its first message, so write() would resend the value left
          // by a previous controller, and the trajectory controller reads it
          // as the current state and would drive the motor back to it. The
          // controller manager calls this before activating any controller.
          // The value is reset before the claim is raised, so that write(),
          // which may be running concurrently, never sees the claim together
          // with the old value.
          if (interface_type == hardware_interface::HW_IF_POSITION)
          {
            if (claimed)
            {
              joints_[i].command.position = kNaN;
            }
            joints_[i].position_claimed = claimed;
          }
          else if (interface_type == hardware_interface::HW_IF_VELOCITY)
          {
            if (claimed)
            {
              joints_[i].command.velocity = kNaN;
            }
            joints_[i].velocity_claimed = claimed;
          }
          else
          {
            break;
          }
          // The released controller's last goal is still running on the
          // controller: have write() stop the motor unless another
          // controller takes the joint over in the same cycle. Raised after
          // the claim is cleared, which write() relies on.
          if (!claimed)
          {
            joints_[i].stop_pending = true;
          }
          break;
        }
      }
    }
  };

  // Order matters if the same interface ever appeared in both lists: a
  // controller switch always releases before it claims, so stop_interfaces
  // is applied first.
  set_claim(stop_interfaces, false);
  set_claim(start_interfaces, true);
  return hardware_interface::return_type::OK;
}

}  // namespace stepit_driver

PLUGINLIB_EXPORT_CLASS(stepit_driver::StepitHardware, hardware_interface::SystemInterface)
