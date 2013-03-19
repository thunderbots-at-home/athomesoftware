#include <opencv2/features2d/features2d.hpp>
#ifndef FEATUREDETECTORPARAMS_CPP
#define FEATUREDETECTORPARAMS_CPP

struct ORBParams_t {
	int nFeatures;
	float scaleFactor;
	int nLevels;
	int edgeThreshold;
	int firstLevel;
	int WTA_K;
	int scoreType;
	int patchSize;

	ORBParams_t () :
	nFeatures(500),
	scaleFactor(1.2f),
	nLevels(8),
	edgeThreshold(31),
	firstLevel(0),
	WTA_K(2),
	scoreType(cv::ORB::HARRIS_SCORE),
	patchSize(31) {}

};

#endif
