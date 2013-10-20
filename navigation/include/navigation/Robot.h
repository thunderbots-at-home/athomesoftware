// Author: Devon Ash
// Thunderbots, UBC
// Robot.h


#ifndef ROBOT_H
#define ROBOT_H

#include "Building.h"
#include "Room.h"

class Robot
{
	private:
	public:
		struct Location robot_location;
		struct Room room; 
		// Include other things like picture of the robot
		// And the models of it for urdf/simulation

		Building building;
};




#endif
