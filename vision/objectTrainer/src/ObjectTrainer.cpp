#include "ObjectTrainer.hpp"
#include "DataCollector.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;
using namespace boost::chrono;
using namespace cv::gpu;

static boost::chrono::steady_clock::time_point start;
static boost::chrono::duration<double> sec;

ObjectTrainer::ObjectTrainer(options_t args) :
_args(args),
_bowide(new BOWImgDescriptorExtractor(new SurfDescriptorExtractor(true), new FlannBasedMatcher())),
_bowTrainer(_args.clusters), // amount of clusters in KNN
_semaphore(args.threads) {} // Semaphore controls how many threads are running at one

// destructor
ObjectTrainer::~ObjectTrainer() {
	_bowide.~Ptr<BOWImgDescriptorExtractor>();
	_semaphore.~interprocess_semaphore();
}

// Public method that gets and prepares all data
// needed for training. Just pass it the root 
// database directory and everything should be
// ready to train.
void ObjectTrainer::getData(string dir) {
	cout<<"Processing images..."<<endl;
	start = steady_clock::now();
	traverseDirectories(dir);
	sec = steady_clock::now() - start;
	int imgs = 0;
	int descriptors = 0;
	//m.lock();
	for(unsigned int i = 0; i < _classes.size(); i++) {
			descriptors += _classes.at(i).getDescriptors().rows;
			imgs += _classes.at(i).getSize();
			_bowTrainer.add(_classes.at(i).getDescriptors());
	}
	//m.unlock();

	if (_args.verbose || _args.verbose_full) {
		cout<<"Processing images complete! Process took "<<sec.count()<<" seconds"<<endl;
		cout<<"Number of classes: "<<_classes.size()<<endl;
		cout<<"Total Images: "<<imgs<<" Total Descriptor: "<<descriptors<<endl;
	}
	getVocab();
	//m.lock();
	getTrainingData();
	//m.unlock();
	train();
	
}

// train the SVM
void ObjectTrainer::train() {
	start = steady_clock::now();
	cout<<"SVM training beginning..."<<endl;
	// SVM_Params defined in definitions.hpp
	_svm.train(_trainingMatrix, _labelMatrix, Mat(), Mat(), SVM_Params);
	sec = steady_clock::now() - start;
	cout<<"SVM training complete. Time taken: "<<sec.count()<<" seconds"<<endl;
	
}

// save trained SVM and data matricies
void ObjectTrainer::save() {

}

//  Create a new DataCollector object for each
//	directory containing images and launch it as a 
//	thread. 
	
void ObjectTrainer::traverseDirectories(string dir) {
	path p (dir);
	vector<path> vec;
	vector<path>::iterator it, it_end;
	boost::thread_group thrds;
	int label = 1;
	if (exists(p)) {
		if (is_directory(p)) {
			if (_args.verbose || _args.verbose_full) {
				cout<<"Entering Directory "<<p<<endl;
			}
			copy(directory_iterator(p), directory_iterator(), back_inserter(vec));
			for(it=vec.begin(), it_end=vec.end(); it != it_end; it++) {
				if (is_directory( (*it) )) {
					_semaphore.wait(); // makes sure the number of threads doesnt exceed a certain number
					boost::thread* thrd = new boost::thread(DataCollector(label, (*it), &_semaphore, _classes, _args));
					thrds.add_thread(thrd);
					label++;
				}
			}
			thrds.join_all();
		}
	}
}


void ObjectTrainer::flannMatching() {

}

// create clusters
void ObjectTrainer::getVocab() {
	cout<<"Creating clusters..."<<endl;
	start = steady_clock::now();
	_vocabMatrix = _bowTrainer.cluster();
	sec = steady_clock::now() - start;
	if (_args.verbose || _args.verbose_full) {
		cout<<"Computer clusters complete. Time taken: "<<sec.count()<<" seconds"<<endl;
	}
}


// load training and label matrix for SVM
void ObjectTrainer::getTrainingData() {
	cout<<"Populating training matrix..."<<endl;
	start = steady_clock::now();
	_bowide->setVocabulary(_vocabMatrix);
	static int ind = 0;
	for (unsigned int i = 0; i < _classes.size(); i++) {
		for (unsigned int j = 0; j < (unsigned)_classes.at(i).getSize(); j++) {
			Mat histResponce;
			_bowide->compute(_classes.at(i).getImage(j),
							 _classes.at(i).getKeypoint(j),
							 histResponce);
			_trainingMatrix.push_back(histResponce);
		}
	}
	_labelMatrix.create(_trainingMatrix.rows, 1, CV_32F);
	for (unsigned int i = 0; i < _classes.size(); i++) {
		for (unsigned int j = 0; j < (unsigned)_classes.at(i).getSize(); j++) {
			_labelMatrix.at<float>(ind, 0) = _classes.at(i).getLabel();
			ind++;
		}
	}
	sec = steady_clock::now() - start;
	if (_args.verbose || _args.verbose_full) {
		cout<<"Creating training matrix complete. Time taken "<<sec.count()<<" seconds"<<endl;
	}
}




