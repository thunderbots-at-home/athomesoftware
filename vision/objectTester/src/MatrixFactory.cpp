#include "definitions.hpp"
#include "MatrixFactory.hpp"

using namespace std;

void MatrixFactory::initFeatureDetector(int featureAlg, cv::Ptr<FeatureDetector>& detector) {
	switch(featureAlg) {
		case f_FAST: {
			detector = new cv::FastFeatureDetector(
				fastParams.threshold,
				fastParams.nonmaxSuppression);
			break;
		}
		case f_STAR: {
			
			detector = new cv::StarFeatureDetector(
				starParams.maxSize,
				starParams.responseThreshold,
				starParams.lineThresholdProjected,
				starParams.lineThresholdBinarized,
				starParams.suppressNonmaxSize);
			break;
		}
		case f_SIFT: {
			detector = new cv::SiftFeatureDetector();
			break;
		}
		case f_SURF: {
			detector = new cv::SurfFeatureDetector(
				surfParams.hessianThreshold,
				surfParams.nOctaves,
				surfParams.nOctaveLayers,
				surfParams.extended,
				surfParams.upright);
			break;
		}
		case f_ORB: {
			detector = new cv::OrbFeatureDetector(
				orbParams.nFeatures,
				orbParams.scaleFactor,
				orbParams.nLevels,
				orbParams.edgeThreshold,
				orbParams.firstLevel,
				orbParams.WTA_K,
				orbParams.scoreType,
				orbParams.patchSize);
			break;
		}
		case f_BRISK: {
			//detector = new cv::BriskFeatureDetector(30,3 1);
			break;
		}
		case f_MSER: {
			detector = new cv::MserFeatureDetector();
			break;
		}
		case f_BLOB: {
			detector = new cv::SimpleBlobDetector();
			break;
		}
		default:
			cout<<"Error getting detector algorithm"<<endl;
	}
}


