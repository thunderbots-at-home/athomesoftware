/*
	The class represents an object that processes all 
	images in a single directory. Each directory can be
	processed completely independently for the others,
	so this class is thread-enabled. 

	Each thread traverses a directory and processes all
	the images in it. First, it loads each image into 
	memory and does preprocessing such as resizing and
	histogram equalization. Next, the keypoints and 
	descriptors are extracted using either the CPU or
	GPU. If the GPU is enabled and multiple threads are
	set, one thread will use the GPU while the others use
	the CPU. Once the keypoints and descriptors are 
	extracted, they are pushed into the ClassContainer
	object.
	
*/

#include "DataCollector.hpp"
#include "definitions.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;
using namespace cv::gpu;

typedef boost::interprocess::interprocess_semaphore Semaphore;
static boost::thread::id gpu_TID; // Thread ID of the thread currently using the GPU
static boost::mutex gpu_m; // GPU mutex

DataCollector::DataCollector(int currentLabel, path dir, Semaphore* semaphore, vector<ClassContainer>& classes, options_t args) : 
_args(args),
_semaphore(semaphore),
_label(currentLabel),
_dir(dir),
_class((float)currentLabel, dir.leaf().string()),
_classes(classes),
_detector(new SurfFeatureDetector(700)),
_extractor(new SurfDescriptorExtractor(true)),
_matcher(new FlannBasedMatcher()) {
	
	}


// destructor
DataCollector::~DataCollector() {
	_detector.~Ptr<FeatureDetector>();
	_extractor.~Ptr<DescriptorExtractor>();
	_matcher.~Ptr<DescriptorMatcher>();
}

// entry point for thread
void DataCollector::operator()() {
	processDirectory();
}

// Each thread calls this method
// This method opens up a directory and for each
// file in the directory, it calls processFile
void DataCollector::processDirectory() {
	path p = _dir;
	vector<path>::iterator it, it_end;
	vector<path> vec;
	if (exists(p)) {
		if (is_directory(p)) {
			if (_args.verbose || _args.verbose_full) {
				m.lock();
				cout<<"Entering Directory "<<p<<endl;
				m.unlock();
			}
			if (_args.gpu && gpu_m.try_lock()) { // if GPU enabled and lock is available
				gpu_TID = boost::this_thread::get_id();
			}
			copy(directory_iterator(p), directory_iterator(), back_inserter(vec));
			// for each file in the directory
			for(it = vec.begin(), it_end = vec.end(); it != it_end; it++) {
				if (is_regular_file( (*it) )){
					processFile( (*it) );
				}
			}
		}
	}
	m.lock();
	_classes.push_back(_class);
	m.unlock();
	_semaphore->post(); // notify next thread
	gpu_m.unlock();
}

// This method processes each image in a directory
// Processing includes:
// * Loading the image into memory and doing any preprocessing
// * Extracting keypoints and descriptors
// * Moving the image Mat, keypoints and descriptors into the
// ClassContainer object for this class
void DataCollector::processFile(path filepath) {
	Mat descriptors;
	vector<KeyPoint> keypoints;
	Mat image;
	bool validImage = loadImage(filepath.string(), CV_LOAD_IMAGE_GRAYSCALE, image);
	if (!validImage) {
		return;
	}
	// if GPU enabled and this thread is using GPU
	if (_args.gpu && gpu_TID == boost::this_thread::get_id()) {
		GpuExtract(image, descriptors, keypoints);
	} else {
		extract(image, descriptors, keypoints);
	}
	if (descriptors.rows > 0) {
		if (_args.verbose_full) {
			m.lock();
			cout<<" Processing "<<
			filepath.leaf().string()<<
			"\tDescriptors: "<<descriptors.rows<<
			" cols: "<<descriptors.cols<<
			"\tLabel: "<<_label<<endl;
			m.unlock();
		}
		_class.push_back(image, descriptors, keypoints); // update class object
	}
}

bool DataCollector::loadImage(string filename, int imageType, Mat& image) {
	Mat src = imread(filename, imageType);
	Mat dst;
	if (!src.data){ // if empty or incorrect type
		return false;
	}

	resize(src, dst, Size(NORMALIZED_WIDTH, NORMALIZED_HEIGHT));
	equalizeHist(dst, image);
	if (_args.show_image) {
		// display picture
		namedWindow("Display Window", CV_WINDOW_AUTOSIZE);
		imshow("Display Winwdow", image);
		waitKey(0);
	}
	return true;
}

void DataCollector::extract(const Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints) {
	_detector->detect(image, keypoints);
	_extractor->compute(image, keypoints, descriptors);
}

void DataCollector::GpuExtract(Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints) {
	SURF_GPU surf;
	GpuMat img(image);
	GpuMat GpuDescriptors;
	GpuMat GpuKeyPoints;
	vector<float> fDescriptors;

	surf(img, GpuMat(), GpuKeyPoints, GpuDescriptors);
	surf.downloadKeypoints(GpuKeyPoints, keypoints);
	surf.downloadDescriptors(GpuDescriptors, fDescriptors);
	descriptors = Mat(fDescriptors, true);
	descriptors = descriptors.reshape(0, fDescriptors.size()/128);

}
