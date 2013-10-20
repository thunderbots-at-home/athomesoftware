// Author: Devon Ash
// Thunderbots, UBC
// Room.h

#ifndef ROOM_H
#define ROOM_H

#include <queue>
#include <vector>

// In-house definitions
#include "Building.h"
#include "ObjectLocation.h"
#include "Human.h"
#include "Robot.h"

class Room
{
	private:
		std::priority_queue<Room> connected_rooms;
		std::priority_queue<Room> upper_level_rooms;		
		std::priority_queue<Room> lower_level_rooms;
		std::vector<ObjectLocation> object_locations_in_room;	
		std::vector<Human> humans_in_room;
		std::vector<Robot> robots_in_room;	

		struct Volume volume;

	public:
		bool hasStaircase;
		bool hasElevator;


};



#endif
