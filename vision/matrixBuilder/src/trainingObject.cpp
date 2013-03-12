#include "libraries.hpp"
using namespace std;
using namespace cv;

class trainingObject {

private:
	Mat _descriptors;
	float _label;
	string _name;

public:
	trainingObject(Mat descriptors, float label, string name) {
		_descriptors = descriptors;
		_label = label;
		_name = name;
	}

	Mat getMat() {
		return _descriptors;
	}

	float getLabel() {
		return _label;
	}

	string getName() {
		return _name;
	}

	void push_back(Mat descriptors) {
		_descriptors.push_back(descriptors);
	}
};

