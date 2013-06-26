// ObjectRecognizer.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================

#include <object_recognition_experimental_v1/ObjectRecognizer.hpp>
#include "MultiObjectRecognition.cpp"
#include "RealWorldObject.cpp"
#include "Scene.cpp"
#include "ObjectLocalizer.cpp"

using namespace std;
using namespace cv;

ObjectRecognizer * recognizer;
string CURRENT_DIRECTORY;
string ObjectRecognizer::getClassName(string& directory)
{

	// Returns last directory name
	istringstream iss(directory);
	string token;
	string last;
	while (getline(iss, token, '/'))
	{
		last = token;
	}
	return last;
}

void ObjectRecognizer::normalize(Mat& input_image, Mat& output_image)
{
	if (!input_image.data) { ROS_INFO("Error with input image"); return ; }
	
	// 3. DoG bandpass filter to avoid aliasing.
	Mat dog_mat;
	Mat dog1, dog2;
	GaussianBlur(input_image, dog1, Size(11, 11), 0);
	GaussianBlur(input_image, dog2, Size(151, 151), 0);
	dog_mat = (dog1 - dog2);

	// 4. Normalize image to standard resolution
	resize(dog_mat, output_image, Size(NORMALIZED_HEIGHT, NORMALIZED_WIDTH)); 
}

void ObjectRecognizer::generateClassLabels(int argc, char** argv)
{
	for (int k = 0; k < argc; k++)
	{
		string d(argv[k]);
		ROS_INFO("Response %d is %s", k, argv[k]);
		this->class_labels.insert( pair < int, string >(k, this->getClassName(d) ));
	}
}

ObjectRecognizer::~ObjectRecognizer()
{

}

ObjectRecognizer::ObjectRecognizer()
{

}

// Input: The number of classes and the class labels [Directory names in this case]
ObjectRecognizer::ObjectRecognizer(int argc, char** argv, Ptr<FeatureDetector>& fdetector, Ptr<DescriptorExtractor>& dextractor, Ptr<DescriptorMatcher>& dmatcher)
{

	this->extractor = dextractor;
	this->matcher = dmatcher;
	this->detector = fdetector;
	SVM_SAVE_NAME = CURRENT_DIRECTORY + "/config/TrainedSVM";
	VOCABULARY_SAVE_NAME = CURRENT_DIRECTORY + "/config/vocabulary";
	
	this->generateClassLabels(argc, argv);
}


void ObjectRecognizer::startStream()
{
	// Load SVM
	// Load vocabulary
	// Begin normalizing images.
	CvSVM svm;
	ROS_INFO("Starting video stream..");
	ROS_INFO("Loading %s SVM", SVM_SAVE_NAME.c_str());
	svm.load(SVM_SAVE_NAME.c_str());
	ROS_INFO("SVM Loaded!");

	ROS_INFO("Loading vocabulary %s.yml", VOCABULARY_SAVE_NAME.c_str());
	Mat vocabulary;		
	FileStorage fs("vocabulary.yml", FileStorage::READ);
	fs["vocabulary"] >> vocabulary;
	BOWImgDescriptorExtractor bowide(this->extractor, this->matcher);
	bowide.setVocabulary(vocabulary);
	ROS_INFO("Vocabulary set");
	ROS_INFO("Vocab type %d", bowide.descriptorType());

	// Load the mapped classes
	
	VideoCapture capture(0);
	if(!capture.isOpened()) return;
	cv::Mat mImage;
	int framecount = 0;

	while(1)
	{
	   	capture >> mImage;
		// Process image
		Mat normalized;
		normalize(mImage, normalized);
		vector<KeyPoint> keypoints;
		detector->detect(normalized, keypoints);

		Mat response;
		bowide.compute(normalized, keypoints, response);
		
		if (!response.data) { ROS_INFO("Invalid response %d", keypoints.size()); } 
		else 
		{
			float resp = svm.predict(response);

			if (resp != 0) 
			{
				framecount++;
				string class_name = this->class_labels.at((int)resp).c_str();
				// Add objects to the scene if they weren't already there.
				RealWorldObject object(mImage, class_name);
				
				if (!Scene::isInScene(object))
				{
					ROS_INFO("Detected object of class %s in stream", class_name.c_str());
					Scene::objects_in_scene.push_back(object);
				}	

				for (int i = 0; i < Scene::objects_of_interest.size(); i++)
				{
					RealWorldObject object = Scene::objects_of_interest[i]; 
					// An Inner loop. (n^2) algorithm, assuming number of objects in scene
					// and number of interesting objects are relatively small.
					// If it is the case that the object counts are becoming too large,
					// then we can switch the scenery to hash maps.

					if (Scene::isInScene(object))
					{
						Mat object_pic = object.getPicture();
						
						imshow("object_pic", object_pic);
						waitKey(30);

						if (object_pic.data)
						{
							ObjectLocalizer localizer(mImage, object_pic, mImage);
							ROS_INFO("Localizing %s", object.getName().c_str());
						} else { ROS_INFO("Object pic null"); }
					}
				}
	
				if (framecount % 30 == 0)
				{
					ROS_INFO("Renewed objects in scene");
					Scene::objects_in_scene.clear();
				}	
			} 
			else 
			{ 
				ROS_INFO("Nothing in stream"); 
			}
		}
		imshow("Img",mImage);
         	waitKey(30);
	}
}

int main(int argc, char** argv)
{

	// Add images to the objects_of_interest thing.
	// Iterate the data folder.
	Scene::loadObjectsOfInterest();

	ros::init(argc, argv, "object_recognizer");
	ros::NodeHandle n;

	char buf[PATH_MAX];

	string dir(getwd(buf));
	CURRENT_DIRECTORY = dir;

	Ptr<FeatureDetector> detector(new SurfFeatureDetector(400) );
	Ptr<DescriptorExtractor> extractor(new SurfDescriptorExtractor);
	Ptr<DescriptorMatcher> matcher(new BruteForceMatcher<L2<float> >());

	ROS_INFO("Creating recognizer...");
	recognizer = new ObjectRecognizer(argc, argv, detector, extractor, matcher);
	recognizer->startStream();
	ROS_INFO("Stream started");

	return 0;
}
