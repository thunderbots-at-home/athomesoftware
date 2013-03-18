#include "definitions.hpp"
#include <iostream>

#ifndef _MATRIXFACTORY_HPP_
#define _MATRIXFACTORY_HPP_

using namespace cv;

class MatrixFactory {

	public:

	void initFeatureDetector(int algIndex, Ptr<FeatureDetector>& detector);

	private:


};

#endif
