#include "definitions.hpp"
#include <iostream>
#include "ObjectTrainer.hpp"


using namespace std;
using namespace cv;

void help() {
	// TODO
}

int main(int argc, char** argv) {

	if (argc != 2) {
		cout << "Expecting one parameter: The database directory" << endl;
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
	



	// TODO descriptor selection

	ObjectTrainer trainer;
	trainer.initialize(argv[1], featureAlg, descriptorAlg);
	trainer.train();
	trainer.save(featureAlg, descriptorAlg);

/*
	clock_t t;
	vector<ClassContainer> classes;
	Mat vocab;
	Mat trainingMatrix;
	Mat labelMatrix;

	MatrixBuilder builder(5);
	t = clock();
	builder.loadClasses(argv[1], classes);
	t = clock() - t;
	cout <<endl<<"It took "<<t/(float)CLOCKS_PER_SEC<<" seconds to finish describing all classes."<<endl;
	t = clock();
	builder.getVocab(vocab);
	t = clock() - t;
	cout <<endl<<"It tool "<<t/(float)CLOCKS_PER_SEC<<" seconds to load the vocabulary."<<endl;
	cout <<endl<<"The vocab has "<<vocab.rows<<" rows"<<endl;
	t = clock();
	builder.getTrainingMatrix(classes, vocab, trainingMatrix, labelMatrix);
	t = clock() - t;
	cout <<endl<<"It took "<<t/(float)CLOCKS_PER_SEC<<" seconds to create the training matrix."<<endl;
	cout <<endl<<"The training matrix contains "<<trainingMatrix.rows<< " rows and the label matrix contains "<< labelMatrix.rows<<" rows" <<endl;
	*/
	return 0;
}



