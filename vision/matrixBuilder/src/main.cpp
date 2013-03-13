#include "MatrixBuilder.hpp"
#include "iostream"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/legacy/compat.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
//	clock_t t;

	if (argc != 2) {
		cout << "Not enough arguements: Please Enter in the database Directory" << endl;
		return -1;
	}

	vector<TrainingObject> classes;

	Mat trainingMatrix(0, 0, CV_32F);
	Mat labelMatrix(0, 0, CV_32F);


	// Build matrices
	MatrixBuilder builder;

	builder.getFiles(argv[1], classes);
	cout << "Total number of classes: " << classes.size() << endl;

	/*
	trainingMatrix.push_back(class1Matrix);
	trainingMatrix.push_back(class2Matrix);

	labelMatrix.push_back(class1Label);
	labelMatrix.push_back(class2Label);

	// BOW
	BOWKMeansTrainer trainer(1000);
	trainer.add(trainingMatrix);
	Mat vocab = trainer.cluster();

	Ptr<DescriptorMatcher > matcher(new FlannBasedMatcher());
	Ptr<DescriptorExtractor > extractor(new OpponentColorDescriptorExtractor(Ptr<DescriptorExtractor>(new SurfDescriptorExtractor())));
	Ptr<BOWImgDescriptorExtractor> bowide(new BOWImgDescriptorExtractor(extractor,matcher));
	bowide->setVocabulary(vocab);
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

	*/
	return 0;
}

	

