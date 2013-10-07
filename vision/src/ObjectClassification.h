// ObjectClassification.h

#ifndef ObjectClassification_H
#define ObjectClassification_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <geometry_msgs/Point>

struct RealObject
{
	Point point;
	Mat picture;
	std::string name;
};

class ObjectClassification
{
	std::vector<RealObject> lastObjectsInScene;

	public: 
	
	std::vector<RealObject>& getObjectsInScene();
	bool containsObject(string name);
	RealObject& findObject(string object);
	RealObject& classify(Mat& img);
		
};



#endif
