#include "definitions.hpp"
#include "MatrixFactory.hpp"

using namespace std;

void MatrixFactory::initFeatureDetector(int algName, cv::Ptr<FeatureDetector>& detector) {
	switch(algName) {
		case mFAST: {
			detector = new cv::FastFeatureDetector();
			break;
		}
		case mSTAR: {
			detector = new cv::StarFeatureDetector();
			break;
		}
		case mSIFT: {
			detector = new cv::SiftFeatureDetector();
			break;
		}
		case mSURF: {
			detector = new cv::SurfFeatureDetector(
				surfParams.hessianThreshold,
				surfParams.nOctaves,
				surfParams.nOctaveLayers,
				surfParams.extended,
				surfParams.upright);
			break;
		}
		case mORB: {
			detector = new cv::OrbFeatureDetector();
			break;
		}
		case mBRISK: {
			//detector = new cv::BriskFeatureDetector(30,3 1);
			break;
		}
		case mMSER: {
			detector = new cv::MserFeatureDetector();
			break;
		}
		case mBLOB: {
			detector = new cv::SimpleBlobDetector();
			break;
		}
		default:
			cout<<"Error getting detector algorithm"<<endl;
	}
}
