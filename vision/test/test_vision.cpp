#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <string>
#include "ObjectClassification.h"

using namespace std;

TEST(Creation, True) 
{
	string topic("/camera/image_raw");
	ObjectClassification oc(topic);
	
	// Is the object created?	
	EXPECT_TRUE(oc.camera_topic.compare("/camera/image_raw") == 0);
}

TEST(Subscribes, True)
{
	string topic("/camera/image_raw");
	ObjectClassification oc(topic);
	
	// Check that the subscriber properly subscribes.
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
