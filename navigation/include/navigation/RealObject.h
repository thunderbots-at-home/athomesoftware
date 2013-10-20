// Author: Devon Ash
// Thunderbots, UBC
// RealObject.h

#ifndef REALOBJECT_H
#define REALOBJECT_H

#include "Building.h"

struct GraspPosition
{
	float left_x;
	float left_y;
	float left_z;

	float right_x;
	float right_y;
	float right_z;
};

struct RealObject
{
	struct GraspPosition grasp_position;
	struct Location location;

};


#endif
