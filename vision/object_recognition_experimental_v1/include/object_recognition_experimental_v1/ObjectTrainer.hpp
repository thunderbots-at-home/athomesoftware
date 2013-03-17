// ObjectTrainer.hpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================

#ifndef __OBJECTTRAINER_H_INCLUDED__
#define __OBJECTTRAINER_H_INCLUDED__

#include "thunderbots_vision.hpp"

using namespace cv;
using namespace std;

struct TrainingRow
{
	string image_name;
	int class_label;
};

class ObjectTrainer
{
	private:

		map< int, string> class_labels;
		TermCriteria KMEANS_TERM_CRITERIA;
		int KMEANS_ATTEMPTS;
		int KMEANS_CLUSTERS;
		int NUM_CLASSES;
		int NUM_IMAGES;
		int NORMALIZED_HEIGHT;
		int NORMALIZED_WIDTH;

		Mat vocabulary;
		Mat trainingMatrix;
		Mat trainingLabels;
		Mat testingMatrix;
		Mat testingLabels;

		CvSVM svm;
		char ** directories;
		int argc;

		Ptr<FeatureDetector> detector;
		Ptr<DescriptorExtractor> extractor;
		Ptr<DescriptorMatcher> matcher;

		vector<TrainingRow*> training_rows;

		char* getFileType(char*);
		string getClassName(string&);
		//void generateClassLabels(int, char**);
		void trainSVM(Mat&, Mat&);
		void normalize(Mat&, Mat&);
		void normalize(string, Mat&);
		void computeTrainingMatrix(Mat&, Mat&, Mat&);
		void computeVocabulary(Mat&);
		void computeHistogram(string, Mat&, Mat&);
		float testSVM();

	public:

		string SVM_SAVE_NAME;
		string VOCABULARY_SAVE_NAME;
		~ObjectTrainer();
		ObjectTrainer();
		ObjectTrainer(int argc, char**, Ptr<FeatureDetector>&, Ptr<DescriptorExtractor>&, Ptr<DescriptorMatcher>&, int, int);
		
};

#endif
