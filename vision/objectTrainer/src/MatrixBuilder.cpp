#include <iostream>
#include "MatrixBuilder.hpp"
#include <boost/filesystem.hpp>
#include "MatrixFactory.hpp"
#include "definitions.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

MatrixBuilder::MatrixBuilder(int featureAlg, string descriptorAlg) : _bowTrainer(186) {
	MatrixFactory factory;
	factory.initFeatureDetector(featureAlg, _detector);
	_extractor = DescriptorExtractor::create(descriptorAlg);
	_matcher = new FlannBasedMatcher();//new BruteForceMatcher<L2 <float> >();
	_bowide = new BOWImgDescriptorExtractor(_extractor, _matcher);
}


MatrixBuilder::~MatrixBuilder() {
	_detector.~Ptr<FeatureDetector>();
	_extractor.~Ptr<DescriptorExtractor>();
	_matcher.~Ptr<DescriptorMatcher>();
}

// set descriptor
void MatrixBuilder::setFeatureDetector( Ptr<FeatureDetector>& detector ) {
	_detector = detector;
}

// Set extractor
void MatrixBuilder::setDescriptorExtractor( Ptr<DescriptorExtractor>& extractor ){
	_extractor = extractor;
}

void MatrixBuilder::setDescriptorMatcher( Ptr<DescriptorMatcher>& matcher ) {
	_matcher = matcher;
}

void MatrixBuilder::setBOWImgDescriptorExtractor( Ptr<BOWImgDescriptorExtractor>& bowide ) {
	_bowide = bowide;
}

void MatrixBuilder::setBOWKMeansTrainer( BOWKMeansTrainer bowTrainer) {
	_bowTrainer = bowTrainer;
}


void MatrixBuilder::extract(const Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints)
{

	_detector->detect(image, keypoints);
	_extractor->compute(image, keypoints, descriptors);

}

void MatrixBuilder::loadClasses(string dir, vector<ClassContainer>& classes) {
	path p (dir);
	static float label = 1.0f;
	vector<path>::iterator it, it_end;
	static ClassContainer obj;
	if (exists(p)) {
		if (is_directory(p)) {
			cout << "Entering Directory " << p << endl;
			t = clock();
			vector<path> vec;
			obj.clear();
			obj.setName(p.leaf().string());
			obj.setLabel(label);
			copy(directory_iterator(p), directory_iterator(), back_inserter(vec));

			for(it = vec.begin(), it_end = vec.end(); it != it_end; ++it){
				loadClasses((*it).string(), classes);
			}
			// TODO Make this better. 
			it = vec.begin(); it_end = vec.end();
			while (it != it_end) {
				if (is_regular_file(*it)) {
					label++;
					//_bowTrainer.add(obj.getDescriptors());
					classes.push_back(obj);
					t = clock() - t;
					cout << endl << "Finished Processing " << obj.getSize()
					<< " images of " << obj.getName() << " in " << ((float)t/CLOCKS_PER_SEC) << " seconds" << endl << endl;
					break;
				}
				++it;
			}

		}
		else if (is_regular_file(p)) {
			Mat image;
			loadImage(p.string(), CV_LOAD_IMAGE_GRAYSCALE, image);
			Mat descriptors;
			vector<KeyPoint> keypoints;
			extract(image, descriptors, keypoints);
			if (descriptors.rows > 0) {
				cout << "\tProcessing " << p.leaf() << "\tNum of Descriptors :"
				<< descriptors.rows << "  \tLabel: " << label << "\tClass Name: " << 
				obj.getName() << endl; 
				_bowTrainer.add(descriptors);
				obj.push_back(image, keypoints);
			}
		}
		else {
			cout << p << " not in dir." << endl;
		}
	}

}

void MatrixBuilder::loadImage(string filename, int imageType, Mat& image) {
	Mat src = imread(filename, imageType);
	assert(src.data);
	image = src;

//	Mat dog1, dog2;
//	GaussianBlur(image, dog1, Size(11,11),0);
//	GaussianBlur(image, dog2, Size(51,51),0);
//	image = dog1 - dog2;

//	equalizeHist(src, image);
	resize(src, image, Size(NORMALIZED_HEIGHT, NORMALIZED_WIDTH));
//	namedWindow("Display Window", CV_WINDOW_AUTOSIZE);
//	imshow("Display Winwdow", image);
//	waitKey(0);
	// TODO -> preprocessing
}

void MatrixBuilder::getVocab(Mat& vocab) {
	vocab = _bowTrainer.cluster();
}

void MatrixBuilder::getTrainingMatrix( vector<ClassContainer>& classes, Mat& vocab, Mat& trainingMatrix, Mat& labelMatrix ) {
	// TODO implement labels
	static int ind=0;
	_bowide->setVocabulary(vocab);
	for (unsigned int i = 0; i < classes.size(); i++) {
		for(unsigned int j = 0; j < unsigned(classes.at(i).getSize()); j++) {
			Mat histResponce;
			vector<KeyPoint> keys = classes.at(i).getKeypoint(j);
			_bowide->compute(classes.at(i).getImage(j), keys, histResponce);
			assert(histResponce.type() == CV_32F);
			trainingMatrix.push_back(histResponce);
		}
	}
	labelMatrix.create(trainingMatrix.rows, 1, CV_32F);
	for (unsigned int i = 0; i < classes.size(); i++) {
		for(unsigned int j = 0; j < unsigned(classes.at(i).getSize()); j++) {
			labelMatrix.at<float>(ind,0) = classes.at(i).getLabel();
			ind++;
		}
	}
}

void MatrixBuilder::predict( vector<ClassContainer>& classes, CvSVM svm)
{
	for(unsigned int i = 0; i < (unsigned int)classes.size(); i++) {
		for(int j = 0; j < classes.at(i).getSize(); j++) {
			vector<KeyPoint> keys = classes.at(i).getKeypoint(j);
			Mat histResponce;
			cout<<classes.at(i).getLabel()<<" ";
			_bowide->compute(classes.at(i).getImage(j), keys, histResponce);
			float result = svm.predict(histResponce, false);
			cout<<classes.at(i).getName()<<" is labeled "<<result<<endl;
		}
	}
}


