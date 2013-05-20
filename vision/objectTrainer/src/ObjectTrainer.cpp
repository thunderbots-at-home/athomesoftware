#include "ObjectTrainer.hpp"
#include "definitions.hpp"

using namespace std;
using namespace cv;

ObjectTrainer::ObjectTrainer() : _trainingMatrix(0, 0, CV_32F), _labelMatrix(0, 0, CV_32F) {
	}

void ObjectTrainer::initialize(string dir, int featureAlg, int descriptorAlg, bool verbose, bool gpu) {
	cout<<"Beginning class loading..."<<endl;
	MatrixBuilder builder(featureAlg, descriptorString(descriptorAlg), verbose);
	_t = clock();
	int totalImgs = 0;
	int totalDesc = 0;
	builder.loadClasses(dir, _classes, totalImgs, totalDesc, gpu);
	_t = clock() - _t;
	cout<<"Class loading Complete! "<<totalImgs<<" Pictures were processed and "<<totalDesc<<" descriptors were extracted"<<endl;
	cout<<"*** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;
	
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
}

void ObjectTrainer::train() {
	cout<<"SVM training beginning..."<<endl;
	_t = clock();
	_svm.train(_trainingMatrix, _labelMatrix, Mat(), Mat(), SVM_Params);
	_t = clock() - _t;
	cout<<"SVM training completed *** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;
}

void ObjectTrainer::train(string dataFileName) {
	FileStorage fs(dataFileName, FileStorage::READ);
	fs["Training Matrix"]>>_trainingMatrix;
	fs["Label Matrix"]>>_labelMatrix;
	cout<<"SVM training beginning..."<<endl;
	_t = clock();
	_svm.train(_trainingMatrix, _labelMatrix, Mat(), Mat(), SVM_Params);
	_t = clock() - _t;
	cout<<"SVM training completed *** Time elapsed : "<<_t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;
}

void ObjectTrainer::save(int featureAlg, int descriptorAlg, bool isLoadOnly, bool timeStamp) {
	string svmFileName;
	string dataFileName;

	time_t timer = time(0);
	struct tm* now = localtime(&timer);
	char buffer[80];
	strftime (buffer, 80, "%F--%X", now);

	svmFileName = "output/svm_";
	svmFileName += featureString(featureAlg) + "_" + descriptorString(descriptorAlg) + "_";
	if (timeStamp)
		svmFileName += buffer;
	svmFileName += ".yaml";
	_svm.save(svmFileName.c_str(), 0);

	dataFileName = "output/trainingData";
	if (timeStamp) {
		dataFileName += "_";
		dataFileName += buffer;
	}
	dataFileName += ".yaml";

	FileStorage fs(dataFileName, FileStorage::WRITE);
	fs<<"Training Matrix"<<_trainingMatrix;
	fs<<"Label Matrix"<<_labelMatrix;
	if (!isLoadOnly){
		fs<< "Vocab Matrix" << _vocab;
	}

	fs.release();
}

string ObjectTrainer::descriptorString( int descriptorAlg) {
	string descriptorName;
	switch (descriptorAlg) {
		case 1: {
			descriptorName = "SIFT";
			break;
		}
		case 2: {
			descriptorName = "SURF";
			break;
		}
		case 3: {
			descriptorName = "ORB";
			break;
		}
		case 4: {
			descriptorName = "BRISK";
			break;
		}
		case 5: {
			descriptorName = "BRIEF";
			break;
		}
		case 6: {
			descriptorName = "OpponentSIFT";
			break;
		}
		case 7: {
			descriptorName = "OpponentSURF";
			break;
		}
		case 8: {
			descriptorName = "OpponentORB";
			break;
		}
		case 9: {
			descriptorName = "OpponentBRISK";
			break;
		}
		case 10: {
			descriptorName = "OpponentBRIEF";
			break;
		}
		default:
			cout<<"Invalid algorithm"<<endl;
	}
	return descriptorName;
}
			
string ObjectTrainer::featureString(int featureAlg){
	string featureName;
	switch (featureAlg) {
		case f_FAST: {
			featureName += "FAST";
			break;
		}
		case f_STAR: {
			featureName += "STAR";
			break;
		}
		case f_SIFT: {
			featureName += "SIFT";
			break;
		}
		case f_SURF: {
			featureName += "SURF";
			break;
		}
		case f_ORB: {
			featureName += "ORB";
			break;
		}
		case f_BRISK: {
			featureName += "BRISK";
			break;
		}
		case f_MSER: {
			featureName += "MSER";
			break;
		}
		case f_BLOB: {
			featureName += "BLOB";
			break;
		}
		default:
			cout<<"Invalid algorithm";
	}

	return featureName;
}

