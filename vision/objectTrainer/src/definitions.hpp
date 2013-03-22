#include <iostream>
#include <string>
#include <iterator>
#include <ctime>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/legacy/compat.hpp>
#include "FeatureDetectorParams.cpp"

#ifndef DEFINITIONS_HPP_
#define DEFINITIONS_HPP_

// Picture sizes
static const int NORMALIZED_HEIGHT = 640;
static const int NORMALIZED_WIDTH = 480;

// Feature Descriptor algorithm Enums

enum featureType { f_FAST = 1, f_STAR = 2, f_SIFT = 3, f_SURF = 4, f_ORB = 5, f_BRISK = 6, f_MSER = 7, f_BLOB = 8 }; 

// Descriptor Extractor algorithm Enums

enum extractorType { e_SIFT = 1, e_SURF = 2, e_BRIEF = 3, o_SIFT = 4, o_SURF = 5, o_BRIEF = 6 };

// SURF Parameters
static const SURFParams_t surfParams(500);

// STAR parameters
static const STARParams_t starParams;

// FAST parameters
static const FASTParams_t fastParams(4, true);

// ORB parameters
static const ORBParams_t orbParams;

// SVM PARAMS
static CvSVMParams SVM_Params;

#endif

