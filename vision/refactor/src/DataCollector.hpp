#include "definitions.hpp"
#include "ClassContainer.cpp"

#ifndef DATACOLLECTOR_HPP
#define DATACOLLECTOR_HPP

using namespace std;
using namespace cv;
using namespace boost::filesystem;
typedef boost::interprocess::interprocess_semaphore Semaphore;


class DataCollector {

	public:
	DataCollector(int currentLabel, path dir, Semaphore* semaphore, vector<ClassContainer>& classes, options_t args);
	~DataCollector();

	void operator()();	

	private:
	options_t _args;
	Semaphore* _semaphore;
	int _label;
	path _dir;
	ClassContainer _class;
	vector<ClassContainer>& _classes;
	Ptr<FeatureDetector> _detector;
	Ptr<DescriptorExtractor> _extractor;
	Ptr<DescriptorMatcher> _matcher;

	void processDirectory();
	void processFile(path filepath);
	bool loadImage(string filename, int imageType, Mat& image);
	void extract(const Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints);
	void GpuExtract(Mat& image, Mat& descriptors, vector<KeyPoint>& keypoints);
};

#endif
