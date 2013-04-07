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

struct FASTParams_t {
	int threshold;
	bool nonmaxSuppression;
	
	FASTParams_t () : threshold(1), nonmaxSuppression(true) {}

	FASTParams_t (int t, bool s) : threshold(t), nonmaxSuppression(s) {}
};

struct SURFParams_t {
	double hessianThreshold; // Threshold for hessian keypoint detector used in SURF.
	int nOctaves; // Number of pyramid octaves the keypoint detector will use. (DEFAULT = 4)
	int nOctaveLayers; // Number of octave layers within each octave. (DEFAULT = 2)
	bool extended;// Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors). (DEFAULT = TRUE)
	bool upright; // Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation). (DEFAULT = FALSE)

	SURFParams_t(double HESSIANTHRESHOLD) : 
	hessianThreshold(HESSIANTHRESHOLD),
	nOctaves(4),
	nOctaveLayers(2),
	extended(true),
	upright(false) {}

	SURFParams_t(double HESSIANTHRESHOLD, int NOCTAVES, int NOCTAVELAYERS, bool EXTENDED, bool UPRIGHT) : 
	hessianThreshold(HESSIANTHRESHOLD),
	nOctaves(NOCTAVES),
	nOctaveLayers(NOCTAVELAYERS),
	extended(EXTENDED),
	upright(UPRIGHT) {}

};

struct STARParams_t {
	int maxSize;
	int responseThreshold;
	int lineThresholdProjected;
	int lineThresholdBinarized;
	int suppressNonmaxSize;

	STARParams_t() : 
	maxSize(16),
	responseThreshold(30),
	lineThresholdProjected(10),
	lineThresholdBinarized(8),
	suppressNonmaxSize(5) {}

	STARParams_t(int MS, int RT, int LTP, int LTB, int SNM) :
	maxSize(MS),
	responseThreshold(RT),
	lineThresholdProjected(LTP),
	lineThresholdBinarized(LTB),
	suppressNonmaxSize(SNM) {}

};


#endif
