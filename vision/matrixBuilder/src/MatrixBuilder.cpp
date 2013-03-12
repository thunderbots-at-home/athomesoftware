#include <iostream>
#include "MatrixBuilder.hpp"
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
using namespace boost::filesystem;

MatrixBuilder::MatrixBuilder() {
	_detector = new SurfFeatureDetector(500);
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

bool MatrixBuilder::loadImages(char* dir, int imageType, vector<Mat>& imgMats){
	
	string dirp = dir;
	vector< vector<trainingObject> > files;
	getFiles(dirp, files);

	cout << files.size() << endl;
	Mat image;
	/*
	// directory accessing
	DIR* dp;
	struct dirent* dirp;
	char* tok;
	string path = dir;
	string fileName;

	if ( (dp = opendir(dir)) == NULL)
	{
		cout << "Error opening Directory" << endl;
		return false;
	}

	while ( (dirp = readdir(dp)) != NULL)
	{
		if (strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
			continue;
		fileName = dirp->d_name;
		path = dir;
		// parse string using "." as delimeter
		tok = strtok(dirp->d_name, ".");
		tok = strtok(NULL, ".");
		if (strcmp(tok, "jpg") == 0 || strcmp(tok, "JPG") == 0)
		{
			image = imread(path + fileName, imageType);
			if (!image.data){
				cout << "Error Reading image " << fileName << endl;
				continue;
			}
			else
			{
				imgMats.push_back(image);
			}
		}
	}
	closedir(dp);
	cout << "Loaded " << imgMats.size() << " pictures." << endl;
	*/
	return true;

}

bool MatrixBuilder::createMatrix(float label, vector<Mat>& imgMats, Mat& mClassData, Mat& mClassLabel, vector<Mat>& mClassVector)
{
	cout << "*** Computing Descriptors ***" << endl;
	int numImgs = imgMats.size();
	vector<KeyPoint> keypoints;
	Mat descriptors;

	for (int i = 0; i < numImgs; i++)
	{
		_detector->detect(imgMats[i], keypoints);
		_extractor->compute(imgMats[i], keypoints, descriptors);
		cout << "Number of descriptors in image " << i << " :  " << descriptors.rows <<  endl;
		Mat labels(descriptors.rows, 1, CV_32F);
		for (int j = 0; j < descriptors.rows; j++)
		{
			labels.at<float>(j,0) = label;
		}
		// load matrices
		mClassLabel.push_back(labels);
		mClassData.push_back(descriptors);
		mClassVector.push_back(descriptors);
	}

	cout << endl << "DONE!" << endl << "Total Descriptors : " << mClassData.rows << endl << endl << endl;
	return true;
}

bool MatrixBuilder::createTest( string file, int imageType, Mat& mTestData)
{
	Mat img = imread(file, imageType);
	if (!img.data)
	{
		cout << "Error reading test image." << endl;
		return false;
	}
	vector<KeyPoint> keypoints;

	_detector->detect(img, keypoints);
	_extractor->compute(img, keypoints, mTestData);

	cout << "Number of descriptors in test image " << file << " : " << mTestData.rows << endl;

	return true;
}

void MatrixBuilder::getFiles(string dir, vector< vector<trainingObject> >& classes) {
	path p (dir);
	static int label = 1;
	vector<path>::iterator it, it_end;
	static vector<trainingObject> objs;
	if (exists(p)) {
		if (is_directory(p)) {
			cout << "Entering Directory " << p << endl;
			vector<path> vec;
			objs.clear();
			copy(directory_iterator(p), directory_iterator(), back_inserter(vec));
		
			for(it = vec.begin(), it_end = vec.end(); it != it_end; ++it){
				getFiles((*it).string(), classes);
						
			}
			// TODO Make this better. 
			it = vec.begin(); it_end = vec.end();
			while (it != it_end) {
				if (is_regular_file(*it)) {
					label++;
					classes.push_back(objs);
					break;
				}
				++it;
			}

		}
		else if (is_regular_file(p)) {
			trainingObject obj(imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE), label, p.parent_path().filename().string());
//			cout << obj.getName() << " - Num of descriptors: " << obj.getMat().rows << " Label : " << obj.getLabel() << endl;
			objs.push_back(obj);
		}
		else {
			cout << p << " not in dir." << endl;
		}
	}
}



