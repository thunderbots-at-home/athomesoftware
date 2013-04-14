#include "ObjectTester.hpp"
#include "definitions.hpp"
#include "MatrixTester.hpp"
#include <ctime>

using namespace std;
using namespace cv;

void ObjectTester::predict(string dir, int featureAlg, int descriptorAlg) {
	MatrixTester tester(featureAlg, descriptorString(descriptorAlg), _vocab);
	clock_t t = clock();
	tester.predict(dir, _svm);
	t = clock() - t;
	cout<<"Total time to predict: "<<t/(float)CLOCKS_PER_SEC<<" seconds"<<endl;
}

void ObjectTester::load(string svmFileName, string dataFileName) {
	_svm.load(svmFileName.c_str(), 0);
	FileStorage fs(dataFileName, FileStorage::READ);
	fs["Vocab Matrix"]>>_vocab;
	cout<<_vocab.rows<<endl;

}

string ObjectTester::descriptorString( int descriptorAlg) {
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
			
string ObjectTester::featureString(int featureAlg){
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
