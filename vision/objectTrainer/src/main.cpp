#include "libraries.hpp"
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

	vector<ClassContainer> classes;
	Mat vocab;
	Mat trainingMatrix;

	MatrixBuilder builder;
	builder.loadClasses(argv[1], classes);
	builder.getVocab(vocab);
	builder.getTrainingMatrix(classes, vocab, trainingMatrix);
	
	return 0;
}



