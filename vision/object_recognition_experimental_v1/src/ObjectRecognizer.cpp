// ObjectRecognizer.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================

#include <object_recognition_experimental_v1/ObjectRecognizer.hpp>

using namespace std;
using namespace cv;

ObjectRecognizer * recognizer;

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
	
	SVM_SAVE_NAME = "TrainedSVM";
	VOCABULARY_SAVE_NAME = "vocabulary";
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

	// Load the mapped classes
	
	VideoCapture capture(0);
	if(!capture.isOpened()) return;
	cv::Mat mImage;

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
			ROS_INFO("response %f", resp);		
			
			if (response) 
			{
				// draw box around object.				
			}
		  	
			ROS_INFO("Detected object of class %s in stream", this->class_labels.at((int)resp).c_str());
		}

		imshow("Img", normalized);
         	waitKey(30);
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_recognizer");
	ros::NodeHandle n;

	Ptr<FeatureDetector> detector(new SurfFeatureDetector(400) );
	Ptr<DescriptorExtractor> extractor(new SurfDescriptorExtractor);
	Ptr<DescriptorMatcher> matcher(new BruteForceMatcher<L2<float> >());

	ROS_INFO("Creating recognizer...");
	recognizer = new ObjectRecognizer(argc, argv, detector, extractor, matcher);
	recognizer->startStream();
	ROS_INFO("Stream started");

	return 0;
}
