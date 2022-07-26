#pragma once

#include <cstdint>

namespace stepit_hardware
{
class MotorStatus
{
public:
  MotorStatus() = default;
  [[nodiscard]] int32_t position() const;
  [[nodiscard]] int32_t distance_to_go() const;
  [[nodiscard]] float speed() const;

private:
  int32_t position_;
  int32_t distance_to_go_;
  float speed_;
};
}  // namespace stepit_hardware
