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

#ifndef GUARD_H
#define GUARD_H

/**
 * This class uses RAII to set a volatile boolean variable to true in the
 * constructor and to false in the destructor.
 *
 * Example:
 *
 * Guard guard{boolVariable, false};
 */
class Guard
{
public:
    Guard(volatile bool &var)
    {
        p_var = &var;
        *p_var = true;
    }
    ~Guard()
    {
        *p_var = false;
        p_var = nullptr;
    }

private:
    volatile bool *p_var = nullptr;
};

#endif // GUARD_H