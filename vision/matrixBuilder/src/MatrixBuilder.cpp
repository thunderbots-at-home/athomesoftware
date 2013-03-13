#include <iostream>
#include "MatrixBuilder.hpp"
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
using namespace boost::filesystem;

MatrixBuilder::MatrixBuilder() {
	_detector = new SurfFeatureDetector(600);
	_extractor = new SurfDescriptorExtractor();
}

MatrixBuilder::MatrixBuilder(const string& detectorType) {
	_detector = FeatureDetector::create(detectorType);
	_extractor = DescriptorExtractor::create(detectorType);
}

MatrixBuilder::~MatrixBuilder() {
	_detector.~Ptr<FeatureDetector>();
	_extractor.~Ptr<DescriptorExtractor>();
}
// set descriptor
void MatrixBuilder::setFeatureDetector( Ptr<FeatureDetector>& detector ) {
	_detector = detector;
}

// Set extractor
void MatrixBuilder::setDescriptorExtractor( Ptr<DescriptorExtractor>& extractor ){
	_extractor = extractor;
}


void MatrixBuilder::extract(const Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints)
{

	_detector->detect(image, keypoints);
	_extractor->compute(image, keypoints, descriptors);

}

void MatrixBuilder::loadClasses(string dir, vector<TrainingObject>& classes) {
	path p (dir);
	static int label = 1;
	vector<path>::iterator it, it_end;
	static TrainingObject obj;
	if (exists(p)) {
		if (is_directory(p)) {
			cout << "Entering Directory " << p << endl;
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
					classes.push_back(obj);
					cout << endl << "Finished Processing " << obj.getSize()
					<< " images of type: " << obj.getName() << " Class." << endl << endl;
					break;
				}
				++it;
			}

		}
		else if (is_regular_file(p)) {
			
			//trainingObject obj(imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE), label, p.parent_path().filename().string());
//			cout << obj.getName() << " - Num of descriptors: " << obj.getMat().rows << " Label : " << obj.getLabel() << endl;
			Mat image;
			loadImage(p.string(), CV_LOAD_IMAGE_GRAYSCALE, image);
			Mat descriptors;
			vector<KeyPoint> keypoints;
			extract(image, descriptors, keypoints);
			cout << "\tProcessing " << p.leaf() << "\tNum of Descriptors :"
			<< descriptors.rows << "  \tLabel: " << label << "\tClass Name: " << 
			obj.getName() << endl; 
			obj.push_back(image, keypoints, descriptors);
		}
		else {
			cout << p << " not in dir." << endl;
		}
	}
}

void MatrixBuilder::loadImage(string filename, int imageType, Mat& image) {
	image = imread(filename, imageType);
	// TODO -> preprocessing
}



