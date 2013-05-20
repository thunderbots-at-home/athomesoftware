#include <iostream>
#include <fstream>
#include <string>
#include "stdlib.h"
#include <iterator>
#include <ctime>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/legacy/compat.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>

#ifndef DEFINITIONS_HPP_
#define DEFINITIONS_HPP_

using namespace std;
using namespace cv::gpu;

// Picture sizes
static const int NORMALIZED_HEIGHT = 300;
static const int NORMALIZED_WIDTH = 300;

// SVM PARAMS
static CvSVMParams SVM_Params(CvSVM::C_SVC, CvSVM::POLY,100,1,1,15,0,0,0,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 10000, 1e-6));

static boost::mutex m;

struct options_t {
	bool verbose;
	bool verbose_full;
	bool time_stamp;
	bool gpu;
	bool less_mem;
	bool flann_enable;
	bool show_image;
	string save_path;
	int clusters;
	int threads;
	int image_type;

	options_t () :
	verbose(false),
	verbose_full(false),
	time_stamp(false),
	gpu(false),
	less_mem(false),
	flann_enable(false),
	show_image(false),
	save_path("."),
	clusters(200),
	threads(1),
	image_type(1){}
};

#endif

