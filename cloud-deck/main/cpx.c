/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "cpx.h"

// Create a routing struct
void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route) {
    route->source = source;
    route->destination = destination;
    route->function = function;
    route->version = CPX_VERSION;
}

// Pack a routing struct
void cpxRouteToPacked(const CPXRouting_t* route, CPXRoutingPacked_t* packed) {
    packed->source = route->source;
    packed->destination = route->destination;
    packed->function = route->function;
    packed->version = route->version;
    packed->lastPacket = route->lastPacket;
}

// Unpack a packed routing struct
void cpxPackedToRoute(const CPXRoutingPacked_t* packed, CPXRouting_t* route) {
    if(CPX_VERSION == packed->version)
    {
        route->version = packed->version;
        route->source = packed->source;
        route->destination = packed->destination;
        route->function = packed->function;
        route->lastPacket = packed->lastPacket;
    }
}