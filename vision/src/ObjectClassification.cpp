#include "ObjectClassification.h"

		
ObjectClassification::ObjectClassification()
{
		
}

// The topic is the topic to be listened to, for example
// if we want to create a recognizer for the hand camera
// or the integrated camera, we just have to change
// what topic it is listening on. 
ObjectClassification::ObjectClassification(std::string topic)
{
	this->camera_topic = topic;
	// Read the images from the camera topic.
	// /camera/image_raw for integrated cam
	this->image_subscriber = image_transport.subscribe(this->camera_topic, 1, &ObjectClassification::save_image, this);

}
	


void ObjectClassification::save_image(const sensor_msgs::ImageConstPtr& image)
{
	// do stuff
	// Convert the image to a openCV image and save it in the vector.
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
	}	
	catch (cv_brdige::Exception& e)
	{
		ROS_ERROR("cv_brdige exception: %s", e.what());
		return;
	}

	this->recent_images.push_back(cv_ptr->image));
	cout << "Saved image " << endl;
	if (this->recent_images.size() > 10)
	{
		this->recent_images.clear();
	}
}

vector<RealObject>& ObjectClassification::getObjectsInScene()
{
	// Pass it into classification algorithm
	// Create real objects once classification algorithm determines what it is
	
	// Put into vector and return
	vector<RealObject> vec;
		
	return vec;
}

bool ObjectClassification::containsObject(std::string name)
{
	return true;
}
	
RealObject& ObjectClassification::findObject(std::string object)
{
	struct RealObject object;
	Mat pic;
	object.picture = pic;

	return object;
}
	
RealObject& ObjectClassification::classify(Mat& img)
{
	struct RealObject object;
	Mat pic;
	object.picture = pic;

	return object;
}

