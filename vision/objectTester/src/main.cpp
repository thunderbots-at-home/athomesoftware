#include "definitions.hpp"
#include "MatrixTester.hpp"
#include "ObjectTester.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
if (argc != 4) {
		cout << "Expecting three parameters: The test directory, svm path, vocab path" << endl;
		return -1;
	}


	int featureAlg = 0;
	int descriptorAlg = 0;

	cout<<"Please select a Feature Detection Algorithm"<<endl;
	cout<<"\t\t1) FAST\t\t2) STAR\t\t3) SIFT\t\t4) SURF\n\t\t5) ORB\t\t6) BRISK\t7) MSER\t\t8) BLOB"<<endl;
	
	while(featureAlg < f_FAST || featureAlg > f_BLOB) {
		while(!(cin>>featureAlg)) {
			cin.clear();
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
		}
	}

	cout<<"Please select a Descriptor Extractor Algorithm"<<endl;
	cout<<"\t\t1) SIFT\t\t2) SURF\t\t3) ORB\t\t4) BRISK\t\t5) BRIEF"<<endl;
	cout<<"\t\t6) OpponentSIFT\t7) OpponentSURF\t8) OpponentORB\t9) OpponentBRISK\t10) OpponentBRIEF\t"<<endl;

	while(descriptorAlg < f_FAST || descriptorAlg > f_BLOB) {
		while(!(cin>>descriptorAlg)) {
			cin.clear();
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
		}
	}
	
	ObjectTester tester;
	tester.load(argv[2], argv[3]);
	tester.predict(argv[1], featureAlg, descriptorAlg);

	return 0;
}



