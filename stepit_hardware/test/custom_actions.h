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

#include <gmock/gmock.h>

/**
 * This is a custom action that uses an iterator to take a value from a vector
 * and pass it to a method output parameter.
 */
ACTION_TEMPLATE(SetArgFromVector, HAS_1_TEMPLATE_PARAMS(unsigned, param_index), AND_1_VALUE_PARAMS(p_iterator))
{
  // param_index indicates the position of the output parameter and
  // p_interator is a pointer to a vector iterator.

  static_assert(std::is_same<decltype(p_iterator), std::vector<uint8_t>::iterator*>::value,
                "p_iterator must be a std::vector<uint8_t>::iterator");

  *std::get<param_index>(args) = **p_iterator;
  ++(*p_iterator);
}
