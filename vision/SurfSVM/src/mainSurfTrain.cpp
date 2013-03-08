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

	if (argc != 3){
		std::cout << "Not enough arguments: include the directory of pictures" << std::endl;
		return -1;
	}

	vector<Mat> imgMats;
	vector<Mat> spoonMats;

	Mat trainingMatrix(0,0,CV_32F);
	Mat labelMatrix(0,0,CV_32F);

	Mat spoonMatrix(0,0,CV_32F);
	Mat spoonLabel(0,0,CV_32F);

	// Build matrix
	SurfMatrixBuilder builder;
	builder.loadImages(argv[1], CV_LOAD_IMAGE_GRAYSCALE, imgMats);
	builder.createMatrix(imgMats, 1.0f, trainingMatrix, labelMatrix);

	builder.loadImages(argv[2], CV_LOAD_IMAGE_GRAYSCALE, spoonMats);
	builder.createMatrix(spoonMats, -1.0f, spoonMatrix, spoonLabel);

	trainingMatrix.push_back(spoonMatrix);
	labelMatrix.push_back(spoonLabel);

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


