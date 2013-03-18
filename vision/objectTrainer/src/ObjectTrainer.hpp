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

	void train(string dir);
	

	private:

	vector<ClassContainer> _classes;
	Mat trainingMatrix;
	Mat labelMatrix;
	MatrixBuilder _matrixBuilder;
	CvSVM _svm;
	clock_t _t;
	// TODO add factory class

};

#endif
