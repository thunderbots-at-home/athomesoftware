#include "libraries.hpp"
#include <iostream>

#ifndef _OBJECTTRAINER_HPP_
#define _OBJECTTRAINER_HPP_

using namespace std;
using namespace cv;

class ObjectTrainer {

	public:

	ObjectTrainer(string dir);

	

	private:

	vector<ClassContainer> _classes;
	MatrixBuilder _matrixBuilder;
	CvSVM _svm;
	string _dir;
	// TODO add factory class

};
