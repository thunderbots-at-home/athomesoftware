#include "definitions.hpp"
#include <iostream>
#include "ObjectTrainer.hpp"


using namespace std;
using namespace cv;

void help() {
	// TODO
}

int main(int argc, char** argv) {

	if (argc < 2) {
		cout << "Expecting one parameter: The database directory" << endl;
		return -1;
	}
	
	bool verbose = false;
	bool isLoadOnly = false;
	bool timeStamp = false;

	if (argc > 2) {
		for(int i = 2; i < argc; i++) {
			if (strcmp(argv[i], "-v") == 0){
				verbose = true;
			}
			if (strcmp(argv[i], "-l") == 0){
				isLoadOnly = true;
			}
			if (strcmp(argv[i], "-t") == 0){
				timeStamp = true;
			}
		}
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
	
	
	ObjectTrainer trainer;
	if (!isLoadOnly) {
		trainer.initialize(argv[1], featureAlg, descriptorAlg, verbose);
		trainer.train();
		trainer.save(featureAlg, descriptorAlg, isLoadOnly, timeStamp);
	} else {
		trainer.train(argv[2]);
		trainer.save(featureAlg, descriptorAlg, isLoadOnly, timeStamp);
	}


	return 0;
}



