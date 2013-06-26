// Scene.hpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================
// include guard

#ifndef __SCENE_H_INCLUDED__
#define __SCENE_H_INCLUDED__

#include "thunderbots_vision.hpp"

using namespace std;
using namespace cv;
class Scene
{
	private:		
		
	public:
		static string environment;
		static vector<RealWorldObject> objects_in_scene;
		static vector<RealWorldObject> objects_of_interest;

		static FlannBasedMatcher matcher;
		static SurfDescriptorExtractor extractor;
		static SurfFeatureDetector detector;
		static bool isInScene(RealWorldObject&);
		static void loadObjectsOfInterest();
		static char* getFileType(char*);
};

#endif
