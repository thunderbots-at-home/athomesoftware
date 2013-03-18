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
#include "SURFParams_t.cpp"

#ifndef DEFINITIONS_HPP_
#define DEFINITIONS_HPP_

// Picture sizes
static const int NORMALIZED_HEIGHT = 640;
static const int NORMALIZED_WIDTH = 480;

// Algorithm Enums

enum algType { mFAST = 1, mSTAR = 2, mSIFT = 3, mSURF = 4, mORB = 5, mBRISK = 6, mMSER = 7, mBLOB = 8 }; 

// SURF Parameters
static const SURFParams_t surfParams(500);

#endif

