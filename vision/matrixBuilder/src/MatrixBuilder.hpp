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

		// extract keypoints and descriptors from one image
		void extract(const Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints);

		// Load an image from file
		void loadImage(string filename, int imageType, Mat& image);

	public: 
		// constructors
		MatrixBuilder();
		MatrixBuilder(const string& detectorType);
		
		// Destructor
		~MatrixBuilder();

		// set descriptor
		void setFeatureDetector( Ptr<FeatureDetector>& detector );

		// Set extractor
		void setDescriptorExtractor( Ptr<DescriptorExtractor>& extractor);
		
		// Creates all trainingObjects 
		void loadClasses(string dir, vector<TrainingObject>& classes);
};

#endif
