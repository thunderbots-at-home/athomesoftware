#include "ObjectTrainer.hpp"
#include "definitions.hpp"

using namespace std;
using namespace cv;

ObjectTrainer::ObjectTrainer() : _trainingMatrix(0, 0, CV_32F), _labelMatrix(0, 0, CV_32F) {
	}

void ObjectTrainer::initialize(string dir, int featureAlg, int descriptorAlg) {
	cout<<"Beginning class loading..."<<endl;
	MatrixBuilder builder(featureAlg, descriptorAlg);
	_t = clock();
	builder.loadClasses(dir, _classes);
	_t = clock() - _t;
	cout<<endl<<"Loading all classes complete *** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;
	
	cout<<endl<<"Creating Vocabulary..."<<endl;
	_t = clock();
	builder.getVocab(_vocab);
	_t = clock() - _t;
	cout<<endl<<"Vocabulary created *** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;

	cout<<endl<<"Loading Training Matrix..."<<endl;
	_t = clock();
	builder.getTrainingMatrix(_classes, _vocab, _trainingMatrix, _labelMatrix);
	_t = clock() - _t;
	cout<<endl<<"Training Matrix created *** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;

	cout<<_trainingMatrix.rows<<"\t"<<_trainingMatrix.cols<<endl;
}

void ObjectTrainer::train() {
	cout<<"SVM training beginning..."<<endl;
	_trainingMatrix.convertTo(_trainingMatrix, CV_32F);
	_t = clock();
	_svm.train(_trainingMatrix, _labelMatrix, Mat(), Mat(), SVM_Params);
	_t = clock() - _t;
	cout<<"SVM training completed *** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;
}


void ObjectTrainer::save(int featureAlg, int descriptorAlg) {
	string filename;
	time_t timer = time(0);
	struct tm* now = localtime(&timer);
	char buffer[80];
	strftime (buffer, 80, "%F--%X", now);
	filename = "trainedSVM_";
	switch (featureAlg) {
		case mFAST: {
			filename += "FAST";
			break;
		}
		case mSTAR: {
			filename += "STAR";
			break;
		}
		case mSIFT: {
			filename += "SIFT";
			break;
		}
		case mSURF: {
			filename += "SURF";
			break;
		}
		case mORB: {
			filename += "ORB";
			break;
		}
		case mBRISK: {
			filename += "BRISK";
			break;
		}
		case mMSER: {
			filename += "MSER";
			break;
		}
		case mBLOB: {
			filename += "BLOB";
			break;
		}
		default:
			filename += "N/A";
	}

	filename += "_SURF_";
	filename += buffer;
	filename += ".yaml";
	cout<<filename<<endl;
	_svm.save(filename.c_str(), 0);

}

