#include "definitions.hpp"
#include <iostream>

#ifndef _OBJECTTESTER_HPP
#define _OBJECTTESTER_HPP

using namespace std;
using namespace cv;

class ObjectTester {
	
	public:

	void predict(string dir, int featureAlg, int descriptorAlg);


	void load(string svmPath, string vocabPath);

	private:

	Mat _vocab;
	CvSVM _svm;
	string descriptorString(int descriptorAlg);
	string featureString(int featureAlg);
};
#endif
