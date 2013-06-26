// ObjectLocalizer.hpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================
// include guard

#ifndef __OBJECTLOCALIZER_H_INCLUDED__
#define __OBJECTLOCALIZER_H_INCLUDED__

#include "thunderbots_vision.hpp"


using namespace std;
using namespace cv;

class ObjectLocalizer
{
	private:
	public:
		~ObjectLocalizer();
		ObjectLocalizer();
		ObjectLocalizer(Mat&, Mat&, Mat&);
};

#endif
