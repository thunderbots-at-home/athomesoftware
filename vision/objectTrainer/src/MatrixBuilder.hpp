#include "libraries.hpp"
#include "ClassContainer.cpp"

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
		Ptr<DescriptorMatcher> _matcher;
		BOWKMeansTrainer _bowTrainer;
		Ptr<BOWImgDescriptorExtractor> _bowide;

		// Clock
		clock_t t;

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
		// set Matcher
		void setDescriptorMatcher( Ptr<DescriptorMatcher>& matcher ); 

		void setBOWImgDescriptorExtractor( Ptr<BOWImgDescriptorExtractor>& extractor );

		void setBOWKMeansTrainer( BOWKMeansTrainer bowTrainer);
		
		// Creates all trainingObjects 
		void loadClasses(string dir, vector<ClassContainer>& classes);

		void getVocab(Mat& vocab);

		void getTrainingMatrix(vector<ClassContainer>& classes, Mat& vocab, Mat& trainingMatrix);
};

#endif
