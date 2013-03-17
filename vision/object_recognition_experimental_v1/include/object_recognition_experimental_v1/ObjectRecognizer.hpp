// ObjectRecognizer.hpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================
// include guard
#ifndef __OBJECTRECOGNIZER_H_INCLUDED__
#define __OBJECTRECOGNIZER_H_INCLUDED__
#define NORMALIZED_HEIGHT 256
#define NORMALIZED_WIDTH 256
#include "thunderbots_vision.hpp"
using namespace std;
using namespace cv;



class ObjectRecognizer
{
		
	public:
		map<int, string> class_labels;
		Ptr<DescriptorExtractor> extractor;
		Ptr<FeatureDetector> detector;
		Ptr<DescriptorMatcher> matcher;

		~ObjectRecognizer();
		ObjectRecognizer();
		ObjectRecognizer(int, char**, Ptr<FeatureDetector>&, Ptr<DescriptorExtractor>&, Ptr<DescriptorMatcher>&);
		void normalize(Mat&, Mat&);
		void generateClassLabels(int, char**);
		string getClassName(string&);
		string SVM_SAVE_NAME;
		string VOCABULARY_SAVE_NAME;
		void startStream();

};

#endif
