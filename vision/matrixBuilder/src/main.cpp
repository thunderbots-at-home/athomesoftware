#include "MatrixBuilder.hpp"
#include "iostream"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/legacy/compat.hpp"
using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	bool success;
	clock_t t;

	if (argc != 4) {
		cout << "Not enough arguements: include two directories" << endl;
		return -1;
	}

	vector<Mat> dir1Mats;
	vector<Mat> dir2Mats;

	Mat trainingMatrix(0, 0, CV_32F);
	Mat labelMatrix(0, 0, CV_32F);

	Mat class1Matrix(0, 0, CV_32F);
	vector<Mat> class1Vector;
	Mat class1Label(0, 0, CV_32F);

	Mat class2Matrix(0 ,0 ,CV_32F);
	vector<Mat> class2Vector;
	Mat class2Label(0, 0, CV_32F);

	// Build matrices
	MatrixBuilder builder(argv[3]);
	success = builder.loadImages(argv[1], CV_LOAD_IMAGE_GRAYSCALE, dir1Mats);
	if (!success) {
		cout << "Error loading images" << endl;
		return -1;
	}
	success = builder.createMatrix(1.0f, dir1Mats, class1Matrix, class1Label, class1Vector);
	if (!success) {
		cout << "Error creating matrix." << endl;
		return -1;
	}

	success = builder.loadImages(argv[2], CV_LOAD_IMAGE_GRAYSCALE, dir2Mats);
	if (!success) {
		cout << "Error loading images" << endl;
		return -1;
	}
	success = builder.createMatrix(-1.0f, dir2Mats, class2Matrix, class2Label, class2Vector);
	if (!success) {
		cout << "Error creating matrix." << endl;
		return -1;
	}


	trainingMatrix.push_back(class1Matrix);
	trainingMatrix.push_back(class2Matrix);

	labelMatrix.push_back(class1Label);
	labelMatrix.push_back(class2Label);
	cout << "Beginning training..." << endl;

	// svm
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-6);

	t = clock();

	CvSVM svm;
	svm.train_auto(trainingMatrix, labelMatrix, Mat(), Mat(), params);
	params = svm.get_params();

	t = clock() - t;
	cout << "Time to train: " << t / ((float)CLOCKS_PER_SEC) << " seconds." <<  endl; 

	return 0;
}

	

