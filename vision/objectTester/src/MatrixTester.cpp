#include "MatrixTester.hpp"
#include "definitions.hpp"
#include "MatrixFactory.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

MatrixTester::MatrixTester(int featureAlg, string descriptorAlg, Mat& vocab) {
	MatrixFactory factory;
	factory.initFeatureDetector(featureAlg, _detector);
	_extractor = DescriptorExtractor::create(descriptorAlg);
	Ptr<DescriptorMatcher> _matcher = new FlannBasedMatcher();
	_bowide = new BOWImgDescriptorExtractor(_extractor, _matcher);
	_bowide->setVocabulary(vocab);

}

MatrixTester::~MatrixTester() {
	_detector.~Ptr<FeatureDetector>();
	_extractor.~Ptr<DescriptorExtractor>();
	_bowide.~Ptr<BOWImgDescriptorExtractor>();
}


void MatrixTester::predict(string dir, CvSVM& svm) {
	path p (dir);
	vector<path>::iterator it, it_end;
	static int _label = 1;
	static int correct = 0;
	static int totalCorrect = 0;
	static int totalCount = 0;
	static int count = 0;
	if (exists(p)) {
		if (is_directory(p)) {
			cout << "Entering Directory " << p << endl;
			vector<path> vec;
			copy(directory_iterator(p), directory_iterator(), back_inserter(vec));

			for(it = vec.begin(), it_end = vec.end(); it != it_end; ++it){
				predict((*it).string(), svm);
			}
			it = vec.begin(); it_end = vec.end();
			while (it != it_end) {
				if (is_regular_file(*it)) {
					_label++;
					count = 0;
					correct = 0;
					break;
				}
				++it;
			}

		}
		else if (is_regular_file(p)) {
			Mat image;
			cout<<"Processing "<<p.string()<<endl;
			loadImage(p.string(), image);
			Mat histResponce;
			extract(image, histResponce);
			if (histResponce.data){
				float label = svm.predict(histResponce, false);
				count++;
				totalCount++;
				if (label == _label){
					correct++;
					totalCorrect++;
				}
				cout<<label<<"\t"<<correct/(float)count * 100<<"\% accurate\t"<<totalCorrect/(float)totalCount * 100<<"\% total Accurate"<<endl;
	
			}
		}
		else {
			cout << p << " not in dir." << endl;
		}
	}


}

void MatrixTester::loadImage(string fileName, Mat& image) {
	Mat src = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
	assert(src.data);
	image = src;
	equalizeHist(src, image);
	resize(src, image, Size(NORMALIZED_HEIGHT, NORMALIZED_WIDTH));
	// TODO -> preprocessing

}

void MatrixTester::extract(const Mat& image, Mat& histResponce) {
	vector<KeyPoint> keys;
	_detector->detect(image, keys);
	_bowide->compute(image, keys, histResponce);
}
