#include "ObjectTrainer.hpp"
#include "definitions.hpp"

using namespace std;
using namespace cv;

ObjectTrainer::ObjectTrainer() : _trainingMatrix(0, 0, CV_32F), _labelMatrix(0, 0, CV_32F) {
	}

void ObjectTrainer::initialize(string dir, int featureAlg, int descriptorAlg) {
	cout<<"Beginning class loading..."<<endl;
	MatrixBuilder builder(featureAlg, descriptorString(descriptorAlg));
	_t = clock();
	builder.loadClasses(dir, _classes);
	_t = clock() - _t;
	int desTotal = 0;
	int picsTotal = 0;
	for (int i = 0; i < _classes.size(); i++){
		desTotal += _classes.at(i).getDescriptors().rows;
		picsTotal += _classes.at(i).getSize();
	}
	cout<<endl<<"Loading all classes complete *** "<<picsTotal<<" Pictures processed and "<<desTotal<<" descriptors extracted"<<endl;
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
		case f_FAST: {
			filename += "FAST";
			break;
		}
		case f_STAR: {
			filename += "STAR";
			break;
		}
		case f_SIFT: {
			filename += "SIFT";
			break;
		}
		case f_SURF: {
			filename += "SURF";
			break;
		}
		case f_ORB: {
			filename += "ORB";
			break;
		}
		case f_BRISK: {
			filename += "BRISK";
			break;
		}
		case f_MSER: {
			filename += "MSER";
			break;
		}
		case f_BLOB: {
			filename += "BLOB";
			break;
		}
		default:
			filename += "N/A";
	}

	filename += "_" + descriptorString(descriptorAlg) + "_";
	filename += buffer;
	filename += ".yaml";
	cout<<filename<<endl;
	_svm.save(filename.c_str(), 0);

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
			

