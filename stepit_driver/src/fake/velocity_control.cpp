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

#include <stepit_driver/fake/velocity_control.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace stepit_driver::velocity_control
{

/**
 * @brief Compute the sign of the given number.
 * @param val The number to calculate the sign of.
 * @return The sign of the given number.
 */
float sgn(double val)
{
  if (val > 0)
  {
    return 1.0;
  }
  if (val < 0)
  {
    return -1.0;
  }
  return 0.0;
}

double position(double v_max, double a, double x0, double v0, double v1, double t)
{
  double x;
  v0 = std::clamp(v0, -v_max, v_max);
  v1 = std::clamp(v1, -v_max, v_max);
  double t1 = std::abs(v1 - v0) / a;
  if (t <= t1)
  {
    x = x0 + v0 * t + 0.5 * a * pow(t, 2) * sgn(v1 - v0);
  }
  else
  {
    x = x0 + (v0 - v1) * t1 + 0.5 * a * pow(t1, 2) * sgn(v1 - v0) + v1 * t;
  }
  return x;
}

double velocity(double v_max, double a, double v0, double v1, double t)
{
  double v;
  v0 = std::clamp(v0, -v_max, v_max);
  v1 = std::clamp(v1, -v_max, v_max);
  double t1 = std::abs(v1 - v0) / a;
  if (t <= t1)
  {
    v = v0 + a * t * sgn(v1 - v0);
  }
  else
  {
    v = v1;
  }
  return v;
}

}  // namespace stepit_driver::velocity_control
