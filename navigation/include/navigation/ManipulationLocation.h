// Author: Devon Ash
// Thunderbots UBC
// ManipulationLocation.h

#ifndef MANIPULATIONLOCATION_H
#define MANIPULATIONLOCATION_H

#include "Building.h"
#include "RealObject.h"

struct ManipulationLocation
{
	Location manipulation_spot_a;
	Location manipulation_spot_b;
	RealObject object_to_manipulate;
};



#endif
