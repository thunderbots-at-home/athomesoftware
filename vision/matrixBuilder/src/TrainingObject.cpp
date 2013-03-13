/*
	This class holds all pretraining data for one class (Eg. Spoons).
	The image matrices, keypoints, and descriptors of every spoon image
	are stored in this one class as vectors.
*/
#include "libraries.hpp"
using namespace std;
using namespace cv;

class TrainingObject {

private:
	vector<Mat> v_images; // each entry is one image matrix
	vector< vector<KeyPoint> > v_keypoints; // each entry is the keypoints of one image
	vector<Mat> v_descriptors; // each entry is the descriptors of one image
	float _label; // SVM label for this class
	string _name; // Name of object representing this class
	int _size; // number of images out into this object

public:
	// constructors
	TrainingObject(float label, string name) {
		_label = label;
		_name = name;
		_size = 0;
	}

	TrainingObject() {
		_label = 0;
		_size = 0;
		_name = "";
	}

	Mat getImage(int i) {
		return v_images.at(i);
	}

	vector<Mat> getAllImages() {
		return v_images;
	}
	
	vector<KeyPoint> getKeypoint(int i) { 
		return v_keypoints.at(i);
	}

	vector< vector<KeyPoint> > getAllKeypoints() {
		return v_keypoints;
	}

	Mat getDescriptor(int i) {
		return v_descriptors.at(i);
	}

	vector<Mat> getAllDescriptors() {
		return v_descriptors;
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
	void push_back(Mat image, vector<KeyPoint> keypoints, Mat descriptors) {
		v_images.push_back(image);
		v_keypoints.push_back(keypoints);
		v_descriptors.push_back(descriptors);
		_size++;
	}
	
	// Clear the contents of this object
	void clear() {
		v_images.clear();
		v_keypoints.clear();
		v_descriptors.clear();
		_size = 0;
		_label = 0;
		_name = "";
	}

};

