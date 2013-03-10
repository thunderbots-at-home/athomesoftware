#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/legacy/compat.hpp"
#include "SurfMatrixBuilder.cpp"
#include "dirent.h"
#include <string>
#include <stdio.h>
#include <ctime>

using namespace cv;
using namespace std;

int main (int argc, char* argv[])
{
	bool success;
	clock_t t;

	if (argc != 4){
		std::cout << "Not enough arguments: include the directory of pictures" << std::endl;
		return -1;
	}

	vector<Mat> dir1Mats;
	vector<Mat> dir2Mats;

	Mat trainingMatrix(0, 0, CV_32F);
	Mat labelMatrix(0, 0, CV_32F);

	Mat class1Matrix(0, 0, CV_32F);
	Mat class1Label(0, 0, CV_32F);

	Mat class2Matrix(0 ,0 ,CV_32F);
	Mat class2Label(0, 0, CV_32F);

	Mat testMatrix;
	Mat testLabel;

	// Build matrices
	SurfMatrixBuilder builder;
	success = builder.loadImages(argv[1], CV_LOAD_IMAGE_GRAYSCALE, dir1Mats);
	if (!success) {
		cout << "Error loading images" << endl;
		return -1;
	}
	success = builder.createMatrix(dir1Mats, 1.0f, class1Matrix, class1Label);
	if (!success) {
		cout << "Error creating matrix." << endl;
		return -1;
	}

	success = builder.loadImages(argv[2], CV_LOAD_IMAGE_GRAYSCALE, dir2Mats);
	if (!success) {
		cout << "Error loading images" << endl;
		return -1;
	}
	success = builder.createMatrix(dir2Mats, -1.0f, class2Matrix, class2Label);
	if (!success) {
		cout << "Error creating matrix." << endl;
		return -1;
	}


	trainingMatrix.push_back(class1Matrix);
	trainingMatrix.push_back(class2Matrix);

	labelMatrix.push_back(class1Label);
	labelMatrix.push_back(class2Label);

	// build test
	success = builder.createTest(argv[3], CV_LOAD_IMAGE_GRAYSCALE, testMatrix);

	cout << testMatrix.rows << endl;
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

	// get results;
	float median = 0;
	for (int i = 0; i < testMatrix.rows; i++)
	{
		median += svm.predict(testMatrix.row(i));
	}
	median /= (float)(testMatrix.rows);
	cout << "Median: " << median << endl;
	return 0;
}


