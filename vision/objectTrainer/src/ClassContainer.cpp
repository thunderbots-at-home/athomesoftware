/*
	This class holds all pretraining data for one class (Eg. Spoons).
	The image matrices, keypoints, and descriptors of every spoon image
	are stored in this one class as vectors.
*/

#ifndef CLASSCONTAINER_CPP_
#define CLASSCONTAINER_CPP_

#include "definitions.hpp"
using namespace std;
using namespace cv;

class ClassContainer {

public:

	// constructors
	ClassContainer(float label /* SVM label */, string name /* Class name */) : _descriptors(0, 0, CV_32F) {
		_label = label;
		_name = name;
		_size = 0;
	}

	Mat getImage(int i) {
		return v_images.at(i);
	}

	vector<Mat> getAllImages() {
		return v_images;
	}
	
	vector<KeyPoint>& getKeypoint(int i) { 
		return v_keypoints.at(i);
	}

	vector< vector<KeyPoint> > getAllKeypoints() {
		return v_keypoints;
	}

	Mat getDescriptors() {
		return _descriptors;
	}

	float getLabel() {
		return _label;
	}

	void setLabel(float label) {
		_label = label;
	}

	string getName() {
		return _name;
	}

	void setName(string name) {
		_name = name;
	}

	int getSize() {
		return _size;
	}

	// push back all data obtained from one image into this object
	void push_back(Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints, string& filePath) {
		v_images.push_back(image);
		v_keypoints.push_back(keypoints);
		v_imgPaths.push_back(filePath);
		_descriptors.push_back(descriptors);
		_size++;
	}

	void push_back(Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints) {
		v_images.push_back(image);
		v_keypoints.push_back(keypoints);
		_descriptors.push_back(descriptors);
		_size++;
	}

	/*
	void push_back(Mat& image, vector<KeyPoint>& keypoints) {
		v_images.push_back(image);
		v_keypoints.push_back(keypoints);
		_size++;
	}
	*/

	void push_back(string& filePath) {
		v_imgPaths.push_back(filePath);
	}

	/*
	void push_back(Mat& descriptors) {
		_descriptors.push_back(descriptors);
	}
	*/

	void add_matches(Mat& flannDescriptors){
		_flannDescriptors.push_back(flannDescriptors);
	}

	// Clear the contents of this object
	void clear() {
		v_images.clear();
		v_keypoints.clear();
		v_imgPaths.clear();
		_flannDescriptors.create(0, 0, CV_32F);
		_descriptors.create(0, 0, CV_32F);
		_size = 0;
		_label = 0;
		_name = "";
	}

private:
	vector<Mat> v_images; // each entry is one image matrix
	vector< vector<KeyPoint> > v_keypoints; // each entry is the keypoints of one image
	vector<string> v_imgPaths; // paths for each image
	Mat _descriptors;
	Mat _flannDescriptors; // Matched descriptors between all images in a class
	float _label; // SVM label for this class
	string _name; // Name of object representing this class
	int _size; // number of images processed



};

#endif
