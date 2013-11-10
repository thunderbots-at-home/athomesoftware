// Author: Devon Ash
// Thunderbots, UBC
// AshianMap.h


#ifndef ASHIANMAP_H
#define ASHIANMAP_H

#include <map>
#include <vector>
#include "ros/ros.h"

#include "Building.h"
#include "Human.h"
#include "Room.h"
#include "RealObject.h"
#include "Robot.h"
#include "ObjectLocation.h"



class AshianMap
{

	private:
		// Addressable by street name/number
		std::map<int, Building> buildings;

	public:
		// Methods for ashian map
		// findhuman
		// Searches all the nodes in the graph for the specific human

		// Finds a human inside a building
		// Finds a human inside a room, this should be used to control the head
		// and search around/find them
		Human findHuman(Building b);
		Human findHuman(Room r);
		
		// findRobot
		// Likewise to the human function
		// Searches all the nodes in the graph for the specific robot
		Robot findRobot(Building b);
		Robot findRobot(Room r);

		// findObject
		// Likewise to above
		std::vector<ObjectLocation> getObjectLocations(RealObject ro);
		AshianPath findObject(Building b);
		
		// Search functions for specific things. 
		AshianPath getPath(Room a, ObjectLocation object_location);
		AshianPath getPath(Room a, Room b);
		AshianPath getPath(Building a, Building b);
};



#endif
