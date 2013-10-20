// ObjectClassification.h
// Author: Devon Ash
// contact: noobaca2@gmail.com
#ifndef ObjectClassification_H
#define ObjectClassification_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <string>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

// Images
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <map>

// File navigation
#include <unistd.h>
#include <dirent.h>
#include <fstream>
#include <iostream>

// Messages + Services
#include "vision/Contains.h"
#include "vision/FindObject.h"
#include "vision/RealObject.h"
#include "vision/Match.h"
#include "vision/GetObjectsInScene.h"

struct RealObject
{
	geometry_msgs::Point point;
	cv::Mat picture;
	std::string name;
};

class ObjectClassification
{
	public:

	static const std::map<std::string, std::string> object_class_locations;
	std::map<std::string, cv::Mat> dataset;
	std::vector<cv::Mat> recent_images;
	std::vector<RealObject> lastObjectsInScene;

	std::string camera_topic;
	std::string dataset_directory;

	ObjectClassification();
	~ObjectClassification();
	ObjectClassification(std::string topic);


	// ATTENTION:
	// These values are given during the setup time for competition. 
	static const std::map<std::string, std::string> init_object_class_locations()
	{
		  std::map<std::string, std::string> m;
		  m[std::string("beverage")] = std::string("kitchen");
		  return m;
	}

	// Callback from listening to topic
	void save_image(const sensor_msgs::ImageConstPtr& image);

	// Service
	bool getObjectsInScene(vision::GetObjectsInScene::Request &req, vision::GetObjectsInScene::Response &res);

	// Service
	bool containsObject(vision::Contains::Request &req, vision::Contains::Response &res);

	// Service	
	bool findObject(vision::FindObject::Request &req, vision::FindObject::Response &res);
	
	// Service
	bool match(vision::Match::Request &req, vision::Match::Response &res);

	// Helper fn
	static cv::Mat* convertToCvImage(const sensor_msgs::Image& image);

	// Load dataset 
	void loadDataset(std::string dataset_directory);
};

#endif
