// ObjectTracker.hpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================
// include guard
#ifndef __OBJECTTRACKER_H_INCLUDED__
#define __OBJECTTRACKER_H_INCLUDED__

#include "thunderbots_vision.hpp"

using namespace std;
using namespace cv;

class ObjectTracker
{

	private:
	public:
	~ObjectTracker();
	ObjectTracker();
	ObjectTracker(Mat&);
	static bool robotShouldTrack();
};
#endif
