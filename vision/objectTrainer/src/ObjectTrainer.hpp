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

	void initialize(string dir, int featureAlg, int descriptorAlg);
	
	void train();

	void save(int featureAlg, int descriptorAlg);
	

	private:

	vector<ClassContainer> _classes;
	Mat _trainingMatrix;
	Mat _labelMatrix;
	Mat _vocab;
	CvSVM _svm;
	clock_t _t;

	// TODO add factory class

};

#endif
