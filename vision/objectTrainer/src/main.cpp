/*
	Main file for ObjectTrainer
*/

#include "definitions.hpp"
#include "ObjectTrainer.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

// return the integer after an "=" in an arg
int get_int_arg(char* arg) {
	char* temp = strtok(arg, "=");
	temp = strtok(NULL, "=");
	return atoi(temp);
}

// Toggle arguements entered
bool set_options(int argc, char** argv, options_t& options) {
	for(int i = 2; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			options.verbose = true;
		} 
		else if (strcmp(argv[i], "-vv") == 0) {
			options.verbose_full = true;
		} 
		else if (strcmp(argv[i], "-t") == 0) {
			options.time_stamp = true;
		} 
		else if (strcmp(argv[i], "-g") == 0) {
			options.gpu = true;
		} 
		else if (strcmp(argv[i], "-m") == 0) {
			options.less_mem = true;
		}
		else if (strcmp(argv[i], "-f") == 0) {
			options.flann_enable = true;
		}	
		else if (strcmp(argv[i], "-S") == 0) {
			if (i+1 >= argc) {
				cout<<"Input error: No specified save directory"<<endl;
				return false;
			} else {
				options.save_path = argv[i+1];
			}
		}
		else if (strstr(argv[i], "--threads=") != NULL) {
			int threads = get_int_arg(argv[i]);
			if (threads > 0) {
				options.threads = threads;
			} else {
				cout<<"Error: Invalid number of threads."<<endl;
				return false;
			}
		}
		else if (strstr(argv[i], "--clusters=") != NULL) {
			int clusters = get_int_arg(argv[i]);
			if (clusters > 0) {
				options.clusters = clusters;
			} else {
				cout<<"Error: Invalid number of clusters."<<endl;
				return false;
			}
		}
		else if (strstr(argv[i],"--image_type=") != NULL) {
			options.image_type = get_int_arg(argv[i]);
		}
		else {
			cout<<"Unknown arguement "<<argv[i]<<endl;
			return false;
		}
	}
	return true;
}

void help() {

}


int main(int argc, char** argv){

	if (argc < 2){
		cout<<"Error: not a valid number of arguements."<<endl;
		help();
		return 1;
	}

	options_t args;
	if (argc > 2) {
		bool status = set_options(argc, argv, args);
		if (!status)
			return 1;
	}

	cout<<"ObjectTrainer program started."<<endl;

	ObjectTrainer ot(args);
	ot.getData(argv[1]);
	return 0;
}

