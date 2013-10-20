// Author: Devon Ash
// Thunderbots, UBC
// Building.h

#ifndef BUILDING_H
#define BUILDING_H

#include <queue>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

struct Volume
{
	float length;
	float width;
	float height;
};

struct Location
{
	// Grid positions
	float x;
	float y;
};

class Building
{

	private:
		std::priority_queue<Building> connected_buildings;
	public:
		struct Volume volume;
		struct Location location;
		cv::Mat picture;

};

#endif
