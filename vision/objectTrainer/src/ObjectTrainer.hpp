#include "definitions.hpp"
#include "ClassContainer.cpp"

#ifndef OBJECTTRAINER_HPP
#define OBJECTTRAINER_HPP

using namespace std;
using namespace cv;
using namespace boost::filesystem;

class ObjectTrainer {

	public:

	ObjectTrainer();
	ObjectTrainer(options_t args);
	~ObjectTrainer();

	void getData(string dir);
	void train();
	void save();

	private:
	options_t _args;
	vector<ClassContainer> _classes;
	 Mat _trainingMatrix;
	 Mat _labelMatrix;
	 Mat _vocabMatrix;
	 Mat _flannMatrix;
	 CvSVM _svm;
	 Ptr<BOWImgDescriptorExtractor> _bowide;
	 BOWKMeansTrainer _bowTrainer;
	 boost::interprocess::interprocess_semaphore _semaphore;
	 

	 void traverseDirectories(string dir);
	 void flannMatching();
	 void getVocab();
	 void getTrainingData();

};





#endif
