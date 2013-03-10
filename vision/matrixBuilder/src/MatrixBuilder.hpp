#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <string>
#include <dirent.h>

#ifndef _MATRIXBUILDER_HPP_
#define _MATRIXBUILDER_HPP_

using namespace std;
using namespace cv;

class MatrixBuilder {

	private:
		// detector and extractor pointers
		Ptr<FeatureDetector> _detector;
		Ptr<DescriptorExtractor> _extractor;

	public: 
		// constructors
		MatrixBuilder();
		MatrixBuilder(const string& detectorType);

		~MatrixBuilder();

		// set descriptor
		void setFeatureDetector( Ptr<FeatureDetector>& detector );

		// Set extractor
		void setDescriptorExtractor( Ptr<DescriptorExtractor>& extractor);

		// Loads all imgs in a directory into a vector of matrices.
		// Each element in the vector is one image matrix
		bool loadImages(
		/*INPUT*/  char* dir, // string to directory
		/*INPUT*/  int imageType, // format of image (eg CV_LOAD_IMAGE_GRAYSCALE)
		/*OUTPUT*/ vector<Mat>& imgMats); // output vector of matrices for each image

		// Calculates all descriptors for each image
		bool createMatrix(
		/*INPUT*/  float label, // class label
		/*INPUT*/  vector<Mat>& imgMats, // vector of image matricies
		/*OUTPUT*/ Mat& mClassData, // matrix of all descriptors 
		/*OUTPUT*/ Mat& mClassLabel, // label matrix for descriptors
		/*OUTPUT*/ vector<Mat>& mClassVector); // vector of descriptors per image
			
		// Create matrix of descriptors for one image
		bool createTest(
		/*INPUT*/  string file, // file to be loaded
		/*INPUT*/  int imageType, // opencv matrix type
		/*OUTPUT*/ Mat& mTestData); // matrix of descriptors

};

#endif
