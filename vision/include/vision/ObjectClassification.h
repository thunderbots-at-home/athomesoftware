// ObjectClassification.h

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

	ros::NodeHandle nh;
	image_transport::Subscriber image_subscriber;
	image_transport::ImageTransport image_transport;

	ObjectClassification();
	~ObjectClassification();
	ObjectClassification(std::string topic);

	// Callback from listening to topic
	void save_image(sensor_msgs::ImageConstPtr& image);

	// Service
	std::vector<RealObject> getObjectsInScene();

	// Service
	bool containsObject(std::string name);

	// Service	
	RealObject& findObject(std::string object);
	
	// Service
	RealObject& classify(cv::Mat& pic);

};

#endif
