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

#pragma once

#include <algorithm>
#include <cstdlib>
#include <string>

namespace stepit_hardware::test
{

// Trim from start.
static inline std::string ltrim(const std::string& s)
{
  std::string value{ s };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  return value;
}

// Trim from end.
static inline std::string rtrim(const std::string& s)
{
  std::string value{ s };
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
              value.end());
  return value;
}

// Trim from both ends.
static inline std::string trim(const std::string& s)
{
  std::string value{ s };
  return ltrim(rtrim(value));
}

// To lower case.
static inline std::string to_lower(const std::string& s)
{
  std::string value{ s };
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return std::tolower(ch); });
  return value;
}

/**
 * @brief If there is a global variable RUN_HARDWARE_TESTS set to true,
 * return false (do not skip the test) else true (skip the test).
 * @return True to skip a test, false to execute it.
 */
bool skip_test()
{
  const char* env = std::getenv("RUN_HARDWARE_TESTS");
  if (env)
  {
    return to_lower(trim(std::string{ env })) != "true";
  }
  return true;
}
}  // namespace stepit_hardware::test
