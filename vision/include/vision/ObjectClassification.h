// ObjectClassification.h

#ifndef ObjectClassification_H
#define ObjectClassification_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <geometry_msgs/Point.h>

struct RealObject
{
	geometry_msgs::Point point;
	cv::Mat picture;
	std::string name;
};

class ObjectClassification
{
	std::vector<RealObject> lastObjectsInScene;

	public:

	std::vector<RealObject>& getObjectsInScene();
	bool containsObject(std::string name);
	RealObject& findObject(std::string object);
	RealObject& classify(cv::Mat& img);

};

#endif
