// Author: Devon Ash
// contact: noobaca2@gmail.com

#include "ObjectClassification.h"

namespace enc = sensor_msgs::image_encodings;
		
ObjectClassification::ObjectClassification()
{
		
}

ObjectClassification::~ObjectClassification()
{

}

void ObjectClassification::save_image(const sensor_msgs::ImageConstPtr& image)
{
	// do stuff
	// Convert the image to a openCV image and save it in the vector.
	if (image)
	{

	} else { std::cout << "Nothing " << std::endl; }
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
	}	
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_brdige exception: %s", e.what());
		return;
	}

	this->recent_images.push_back(cv_ptr->image);
	std::cout << "Saved image " << std::endl;
	if (this->recent_images.size() > 10)
	{
		this->recent_images.clear();
	}
}

// Topic: topic to subscribe to for images
ObjectClassification::ObjectClassification(std::string topic)
{
	this->camera_topic = topic;
	// Read the images from the camera topic.
	// /camera/image_raw for integrated cam
	
}

// The getObjectsInScene service callback
std::vector<RealObject> ObjectClassification::getObjectsInScene(vision::GetObjectsInScene::Request &req, vision::GetObjectsInScene::Response &res)
{
	// Pass it into classification algorithm
	// Create real objects once classification algorithm determines what it is
	
	// Put into vector and return
	std::vector<RealObject> vec;
		
	return vec;
}


// The containsObject service callback
bool ObjectClassification::containsObject(vision::Contains::Request &req, vision::Contains::Response &res)
{
	return true;
}
	
// The findobject service callback
RealObject& ObjectClassification::findObject(vision::FindObject::Request &req, vision::FindObject::Response &res)
{
	struct RealObject obj;
	cv::Mat pic;
	obj.picture = pic;

	return obj;
}
	

// The classify service callback
RealObject& ObjectClassification::classify(vision::Classify::Request &req, vision::Classify::Response &res)
{
	struct RealObject object;
	cv::Mat pics;
	object.picture = pics;

	return object;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_classification");
	ros::NodeHandle nh;

	std::string topic;
	nh.getParam("camera_topic", topic);

	ObjectClassification oc(topic);

	// Get the node to subscribe to the image topic
	ros::Subscriber subscriber = nh.subscribe(oc.camera_topic, 1, &ObjectClassification::save_image, &oc);

	// Set up the services for the node. 
	
	ros::spin();
}

