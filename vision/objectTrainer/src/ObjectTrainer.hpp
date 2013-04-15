#include "definitions.hpp"
#include <iostream>
#include "ClassContainer.cpp"
#include "MatrixBuilder.hpp"

#ifndef _OBJECTTRAINER_HPP_
#define _OBJECTTRAINER_HPP_

using namespace std;
using namespace cv;

class ObjectTrainer {

	public:

	ObjectTrainer();

	void initialize(string dir, int featureAlg, int descriptorAlg, bool verbose, bool gpu);
	
	void train();

	void train(string dataFileName);

	void save(int featureAlg, int descriptorAlg, bool isLoadOnly, bool timeStamp);

	

	private:

	vector<ClassContainer> _classes;
	Mat _trainingMatrix;
	Mat _labelMatrix;
	Mat _vocab;
	CvSVM _svm;
	clock_t _t;

	string descriptorString(int descriptorAlg);
	string featureString(int featureAlg);

	// TODO add factory class

};

#endif
