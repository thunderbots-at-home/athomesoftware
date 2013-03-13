#include "libraries.hpp"
#include "TrainingObject.cpp"

#ifndef _MATRIXBUILDER_HPP_
#define _MATRIXBUILDER_HPP_

using namespace std;
using namespace cv;
using namespace boost::filesystem;

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

		void getFiles(string dir, vector<TrainingObject>& classes);
		// Create matrix of descriptors for one image
		void create(const Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints);
		Mat loadImage(string filename, int imageType);
};

#endif
