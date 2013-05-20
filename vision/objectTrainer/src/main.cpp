#include "definitions.hpp"
#include <iostream>
#include "ObjectTrainer.hpp"


using namespace std;
using namespace cv;

void help() {
	cout<<"\n\n\t\t*** Object Training Program ***"<<endl<<endl;
	cout<<"\tTrains an SVM classifier using one of several\n\t feature detection "
			"and extraction algorithms."<<endl;
	cout<<endl<<"OUTPUTS:\n\n\tTwo .yaml files. The first file is the trained SVM\n\t"
			"that can be loaded into the OpenCV SVM for prediction.\n\t"
			"The second file contains several data structures created\n\twhile training,"
			"including the SVM Training Matrix composed\n\tof the image histogram responses,"
			"the BOW vocabulary matrix,\n\tand the corresponding label names"
			"for each response in the SVM."<<endl<<endl;
	cout<<"INPUTS:\n\tMANDATORY: The first arguement must be the path to the\n\t"
		"training folder. The folder must contain subfolders,\n\t"
		"each containing images of a specific class."<<endl<<endl;;
	cout<<"\tOPTIONAL:\n\n\t\t -v\tVerbose\n\n\t\t -l\tLoad Training Matrix. This option allows you\n\t\t\t"
		"to input an already made training Matrix and\n\t\t\trun the SVM training only, "
		"bypassing all image\n\t\t\textraction and vocab calculations.\n\t\t\t"
		"The second arguement after this one must be\n\t\t\t"
		"the path to the data file outputed by this\n\t\t\t"
		"program previously."<<endl<<endl;
	cout<<"\t\t -t\tAdd a time stamp to the file outputs\n\t\t\t"
		"to create unique filenames"<<endl<<endl;;
	cout<<"\t\t -g\tEnable GPU support.\n\t\t\t"
		"The following requirements must be met:\n\t\t\t"
		"NVidia Drivers installed\n\t\t\tCUDA toolkit installed,\n\t\t\t"
		"OpenCV compiled with CUDA enabled"<<endl;

}

int main(int argc, char** argv) {

	if (argc < 2) {
		help();
		return 1;
	}
	
	bool verbose = false;
	bool isLoadOnly = false;
	bool timeStamp = false;
	bool gpu = false;

	if (strcmp(argv[1], "--help") == 0){
		help();
		return 0;
	}

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
			if (strcmp(argv[i], "-g") == 0){
				gpu = true;
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
		trainer.initialize(argv[1], featureAlg, descriptorAlg, verbose, gpu);
		trainer.train();
		trainer.save(featureAlg, descriptorAlg, isLoadOnly, timeStamp);
	} else {
		trainer.train(argv[2]);
		trainer.save(featureAlg, descriptorAlg, isLoadOnly, timeStamp);
	}


	return 0;
}



