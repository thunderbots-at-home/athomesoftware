// Author: Devon Ash
// Thunderbots UBC
// ObjectLocation.h

#ifndef OBJECTLOCATION_H
#define OBJECTLOCATION_H


#include "Building.h"
#include "ManipulationLocation.h"

#include <vector>

class ObjectLocation
{
	private:
		cv::Mat picture;
		std::vector<ManipulationLocation> manipulation_locations;
		std::vector<RealObject> objects_at_location;
	public:
		struct Location location;


};



#endif

