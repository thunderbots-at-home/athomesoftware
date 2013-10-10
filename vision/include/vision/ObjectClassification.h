// ObjectClassification.h
// Author: Devon Ash
// contact: noobaca2@gmail.com
#ifndef ObjectClassification_H
#define ObjectClassification_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "vision/Contains.h"

struct RealObject
{
	geometry_msgs::Point point;
	cv::Mat picture;
	std::string name;
};

class ObjectClassification
{
	public:

	std::vector<cv::Mat> recent_images;
	std::vector<RealObject> lastObjectsInScene;
	std::string camera_topic;

	ObjectClassification();
	~ObjectClassification();
	ObjectClassification(std::string topic);

	// Callback from listening to topic
	void save_image(const sensor_msgs::ImageConstPtr& image);

	// Service
	std::vector<RealObject> getObjectsInScene();

	// Service
	bool containsObject(vision::Contains::Request &req, vision::Contains::Response &res);

	// Service	
	RealObject& findObject(std::string object);
	
	// Service
	RealObject& classify(cv::Mat& pic);

};

#endif
