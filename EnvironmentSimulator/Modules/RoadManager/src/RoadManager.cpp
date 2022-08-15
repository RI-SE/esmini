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

/*
 * This module provides an limited interface to OpenDRIVE files.
 * It supports all geometry types (as of ODR 1.4), junctions and some properties such as lane offset
 * but lacks many features as road markings, signals, road signs, speed and road banking
 *
 * It converts between world (cartesian) and road coordinates (both Track and Lane)
 *
 * When used standalone (outside ScenarioEngine) the road manager is initialized via the Position class like
 *this: roadmanager::Position::LoadOpenDrive("example.xodr");
 *
 * Simplest use case is to put a vehicle on the road and simply move it forward along the road, e.g:
 *
 *   car->pos = new roadmanager::Position(3, -2, 10, 0);
 *   while(true)
 *   {
 *	    car->pos->MoveAlongS(0.1);
 *   }
 *
 * The first line will create a Position object initialized at road with ID = 3, in lane = -2 and at lane
 *offset = 0 Then the position is updated along that road and lane, moving 10 cm at a time.
 *
 * A bit more realistic example:
 *
 *   car->pos = new roadmanager::Position(odrManager->GetRoadByIdx(0)->GetId(), -2, 10, 0);
 *   while(true)
 *   {
 *	    car->pos->MoveAlongS(speed * dt);
 *   }
 *
 * Here we refer to the ID of the first road in the network. And instead of static delta movement, the
 *distance is a function of speed and delta time since last update.
 *
 */

#include <assert.h>
#include <time.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>

#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include "odrSpiral.h"
#include "pugixml.hpp"

static unsigned int global_lane_counter;

using namespace std;
using namespace roadmanager;

#define CURV_ZERO 0.00001
#define MAX_TRACK_DIST 10
#define OSI_POINT_CALC_STEPSIZE 1		 // [m]
#define OSI_TANGENT_LINE_TOLERANCE 0.01	 // [m]
#define OSI_POINT_DIST_SCALE 0.025


static int g_Lane_id = 0;
static int g_Laneb_id = 0;

static std::string LinkType2Str(LinkType type) {
	if (type == LinkType::PREDECESSOR) {
		return "PREDECESSOR";
	} else if (type == LinkType::SUCCESSOR) {
		return "SUCCESSOR";
	} else if (type == LinkType::NONE) {
		return "NONE";
	} else {
		return std::string("Unknown link type: " + std::to_string(type));
	}
}

int roadmanager::GetNewGlobalLaneId() {
	int returnvalue = g_Lane_id;
	g_Lane_id++;
	return returnvalue;
}

int roadmanager::GetNewGlobalLaneBoundaryId() {
	int returnvalue = g_Laneb_id;
	g_Laneb_id++;
	return returnvalue;
}





