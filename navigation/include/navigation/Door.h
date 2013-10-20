// Author: Devon Ash
// Thunderbots, UBC
// Door.h

#ifndef DOOR_H
#define DOOR_H

#include "Room.h"

struct DoorKnobLocation
{
	float x_relative_to_door;
	float y_relative_to_door;
	float z_relative_to_door;
};

class Door
{
	private:
		Room room_a;
		Room room_b;
	public:
		bool isOpen;
		struct DoorKnobLocation door_knob_location;
};



#endif
