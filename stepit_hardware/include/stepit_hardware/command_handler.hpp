/*
 * Copyright (C) 2022 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#pragma once

#include <stepit_hardware/msgs/request.hpp>
#include <stepit_hardware/msgs/response.hpp>
#include <stepit_hardware/data_interface.hpp>

#include <memory>

namespace stepit_hardware
{
class CommandHandler
{
public:
  explicit CommandHandler(std::unique_ptr<DataInterface> data_interface);

  template <class T>
  T send([[maybe_unused]] const Request& request) const
  {
    data_interface_->write(request.bytes());
    auto data = data_interface_->read();
    return T{ data };
  }

private:
  std::unique_ptr<DataInterface> data_interface_;
};
}  // namespace stepit_hardware
