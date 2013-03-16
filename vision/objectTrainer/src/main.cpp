#include "definitions.hpp"
#include <iostream>
#include "MatrixBuilder.hpp"

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

	clock_t t;
	vector<ClassContainer> classes;
	Mat vocab;
	Mat trainingMatrix;

	MatrixBuilder builder;
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
	builder.getTrainingMatrix(classes, vocab, trainingMatrix);
	t = clock() - t;
	cout <<endl<<"It took "<<t/(float)CLOCKS_PER_SEC<<" seconds to create the training matrix."<<endl;
	cout <<endl<<"The training matrix contains "<<trainingMatrix.rows<< " rows"<<endl;
	return 0;
}



