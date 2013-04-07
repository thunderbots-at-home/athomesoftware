#include "definitions.hpp"

#ifndef MATRIXTESTER_CPP_
#define MATRIXTESTER_CPP_

using namespace std;
using namespace cv;
using namespace boost::filesystem;

class MatrixTester {

	public:

		MatrixTester(int featureAlg, string descriptorAlg, Mat& vocab);

		~MatrixTester();

		void predict(string dir, CvSVM& svm);



	private:

		Ptr<FeatureDetector> _detector;
		Ptr<DescriptorExtractor> _extractor;
		Ptr<BOWImgDescriptorExtractor> _bowide;

		void loadImage(string filename, Mat& image);

		void extract(const Mat& image, Mat& histResponce);


};
#endif
