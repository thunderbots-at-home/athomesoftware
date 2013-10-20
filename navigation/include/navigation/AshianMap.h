// Author: Devon Ash
// Thunderbots, UBC
// AshianMap.h


#ifndef ASHIANMAP_H
#define ASHIANMAP_H

#include <map>
#include "ros/ros.h"
#include "Building.h"

class AshianMap
{

	private:
		// Addressable by street name/number
		std::map<int, Building> buildings;

	public:
		// Methods for ashian map
		// findhuman
		// Searches all the nodes in the graph for the specific human
				
		// findRobot
		// Searches all the nodes in the graph for the specific robot
		
		// findPathToRoom
		// 
		// getPathToBuilding
		// getObjectLocations(RealObject object)
		// 
};



#endif
