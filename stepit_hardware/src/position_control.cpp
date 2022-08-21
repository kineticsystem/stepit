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

#include <stepit_hardware/position_control.hpp>

#include <algorithm>

namespace stepit_hardware::kinematics
{

/**
 * @brief Compute the sign of the given number.
 * @param val The number to calculate the sign of.
 * @return The sign of the given number.
 */
float sgn(float val)
{
  if (val > 0)
  {
    return 1.0f;
  }
  if (val < 0)
  {
    return -1.0f;
  }
  return 0.0f;
}

/**
 * @brief Compute the time to stop for a motor with acceleration a and rotating
 * speed v0.
 * @param a The motor acceleration.
 * @param v0 The current speed.
 * @return The time for the motor to stop.
 */
float time_to_stop(float a, float v0)
{
  return std::abs(v0 / a);
}

/**
 * @brief Compute the time for a motor, with acceleration a and zero initial
 * velocity, to move from position x0 to position x1.
 * @param v_max The maximum velocity.
 * @param a     The acceleration.
 * @param x0    The initial position.
 * @param x1    The final position.
 * @return      The motor velocity.
 */
float time_to_go(float v_max, float a, float x0, float x1)
{
  float dx = x1 - x0;
  float total_time = 0;
  if (std::abs(dx) >= powf(v_max, 2) / a)
  {
    float t1 = v_max / a;
    float t2 = t1 + std::max(0.0f, (std::abs(dx) - powf(v_max, 2) / a)) / v_max;
    total_time = t1 + t2;
  }
  else
  {
    float t1 = std::sqrt(std::abs(dx) / a);
    total_time = 2 * t1;
  }
  return total_time;
}

/**
 * @brief Compute the position of a motor with acceleration a and zero initial
 * velocity, moving from position x0 to x1.
 * @param v_max The maximum velocity.
 * @param a     The acceleration.
 * @param x0    The initial position.
 * @param x1    The final position.
 * @param t     The time.
 * @return      The motor position.
 */
float position(float v_max, float a, float x0, float x1, float t)
{
  float dx = x1 - x0;
  float x = 0;
  if (std::abs(dx) >= pow(v_max, 2) / a)
  {
    float t1 = v_max / a;
    float t2 = t1 + std::fmax(0.0f, (std::abs(dx) - powf(v_max, 2) / a)) / v_max;
    float t3 = t1 + t2;
    if (t <= t1)
    {
      x = 0.5f * a * powf(t, 2);
    }
    else if (t <= t2)
    {
      x = 0.5f * powf(v_max, 2) / a + v_max * (t - t1);
    }
    else if (t <= t3)
    {
      x = 0.5f * powf(v_max, 2) / a + v_max * (t - t1) - 0.5f * a * powf(t - t2, 2);
    }
    else
    {
      x = powf(v_max, 2) / a + v_max * (t2 - t1);
    }
  }
  else
  {
    float t1 = std::sqrt(std::abs(dx) / a);
    float t2 = 2 * t1;
    if (t <= t1)
    {
      x = 0.5f * a * powf(t, 2);
    }
    else if (t <= t2)
    {
      x = 0.5f * std::abs(dx) + a * t1 * (t - t1) - 0.5f * a * powf(t - t1, 2);
    }
    else
    {
      x = std::abs(dx);
    }
  }
  return x * sgn(dx);
}

/**
 * @brief Compute the velocity of a motor with acceleration a and zero initial
 * velocity, moving from position x0 to x1.
 * @param v_max The maximum velocity.
 * @param a     The acceleration.
 * @param x0    The initial position.
 * @param x1    The final position.
 * @param t     The time.
 * @return      The motor velocity.
 */
float velocity(float v_max, float a, float x0, float x1, float t)
{
  float v = 0;
  float dx = x1 - x0;
  if (std::abs(dx) >= powf(v_max, 2) / a)
  {
    float t1 = v_max / a;
    float t2 = t1 + std::max(0.0f, (std::abs(dx) - powf(v_max, 2) / a)) / v_max;
    float t3 = t1 + t2;
    if (t <= t1)
    {
      v = a * t;
    }
    else if (t <= t2)
    {
      v = a * t1;
    }
    else if (t <= t3)
    {
      v = a * t1 - a * (t - t2);
    }
    else
    {
      v = 0;
    }
  }
  else
  {
    float t1 = std::sqrt(std::abs(dx) / a);
    float t2 = 2 * t1;
    if (t <= t1)
    {
      v = a * t;
    }
    else if (t <= t2)
    {
      v = a * t1 - a * (t - t1);
    }
    else
    {
      v = 0;
    }
  }
  return v * sgn(dx);
}

float distance_to_stop(float a, float v0)
{
  return 0.5f * powf(v0, 2) / a;
}

float time_to_go(float v_max, float a, float v0, float x0, float x1)
{
  v0 = std::clamp(v0, -v_max, v_max);
  float dx = x1 - x0;
  float d_stop = distance_to_stop(a, v0);
  float t_stop = time_to_stop(a, v0);
  float total_time = 0;
  if ((v0 >= 0 && dx >= d_stop) || (v0 <= 0 && dx <= -d_stop))  // Keep accelerating.
  {
    total_time = time_to_go(v_max, a, x0, x1 + sgn(v0) * d_stop) - t_stop;
  }
  else  // Decelerate and reverse.
  {
    total_time = time_to_go(v_max, a, x0, x1 - sgn(v0) * d_stop) + t_stop;
  }
  return total_time;
}

float position(float v_max, float a, float v0, float x0, float x1, float t)
{
  float x = 0;
  v0 = std::clamp(v0, -v_max, v_max);
  float dx = x1 - x0;
  float d_stop = distance_to_stop(a, v0);
  float t_stop = time_to_stop(a, v0);
  if ((v0 >= 0 && dx >= d_stop) || (v0 <= 0 && dx <= -d_stop))  // Keep accelerating.
  {
    x = position(v_max, a, x0, x1 + sgn(v0) * d_stop, t + t_stop) - sgn(v0) * d_stop;
  }
  else
  {
    if (t <= t_stop)  // Decelerate.
    {
      x = v0 * (t - t_stop) + sgn(v0) * (0.5f * a * (powf(t_stop, 2) - powf(t, 2)) + d_stop);
    }
    else  // Reverse.
    {
      x = position(v_max, a, x0, x1 - sgn(v0) * d_stop, t - t_stop) + sgn(v0) * d_stop;
    }
  }
  return x + x0;
}

float velocity(float v_max, float a, float v0, float x0, float x1, float t)
{
  float v;
  v0 = std::clamp(v0, -v_max, v_max);
  float dx = x1 - x0;
  float d_stop = distance_to_stop(a, v0);
  float t_stop = time_to_stop(a, v0);
  if ((v0 >= 0 && dx >= d_stop) || (v0 <= 0 && dx <= -d_stop))  // Keep accelerating.
  {
    v = velocity(v_max, a, x0, x1 + sgn(v0) * d_stop, t + t_stop);
  }
  else
  {
    if (t <= t_stop)  // Decelerate.
    {
      v = v0 - sgn(v0) * a * t;
    }
    else  // Reverse.
    {
      v = velocity(v_max, a, x0, x1 - sgn(v0) * d_stop, t - t_stop);
    }
  }
  return v;
}

}  // namespace stepit_hardware::kinematics
