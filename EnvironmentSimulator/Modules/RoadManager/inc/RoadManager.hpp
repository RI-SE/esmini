/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <cmath>
#include <list>
#include <map>
#include <string>
#include <vector>
#include "CommonMini.hpp"
#include "pugixml.hpp"
#include "Polynomial.hpp"
#include "Position.hpp"
#include "Road.hpp"
#include "OSI.hpp"

namespace roadmanager {
int GetNewGlobalLaneId();
int GetNewGlobalLaneBoundaryId();

// Forward declarations
class Route;
class RMTrajectory;


// A Road Path is a linked list of road links (road connections or junctions)
// between a starting position and a target position
// The path can be calculated automatically

/**
	This nurbs implementation is strongly inspired by the "Nurbs Curve Example" at:
	https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html
*/


  // namespace roadmanager
