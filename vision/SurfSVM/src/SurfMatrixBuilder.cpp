#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <string>
#include <dirent.h>

using namespace cv;
using namespace std;

class SurfMatrixBuilder {
	
	private:
		// detector and extractor pointers
		Ptr<FeatureDetector> _detector;
		Ptr<DescriptorExtractor> _extractor;
		const int _width;

	public:
		SurfMatrixBuilder() : _width(128)
		{
			_detector = new SurfFeatureDetector(500);
			_extractor = new SurfDescriptorExtractor();
		}

		~SurfMatrixBuilder()
		{
			_detector.~Ptr<FeatureDetector>();
			_extractor.~Ptr<DescriptorExtractor>();
		}

		// Setters
		void setFeatureDetector( Ptr<FeatureDetector>& detector )
		{
			_detector = detector;
		}

		void setDescriptorExtractor( Ptr<DescriptorExtractor>& extractor )
		{
			_extractor = extractor;
		}

		/* Creates a training matrix from image matrices
			INPUT@ imgMats -> vector of image matrices
			OUTPUT@ trainingMatrix -> matrix of image descriptors concatenated into one matrix
		*/
		bool createMatrix( vector<Mat>& imgMats, float label, Mat& trainingMatrix, Mat& labelMatrix ) 
		{		
			cout << "*** Computing Descriptors ***" << endl;
			int numImgs = imgMats.size();
			vector<KeyPoint> keypoints;
			Mat descriptors;

			for (int i = 0; i < numImgs; i++)
			{
				_detector->detect(imgMats[i], keypoints);
				_extractor->compute(imgMats[i], keypoints, descriptors);
				cout << "Number of descriptors in image " << i << " :  " << descriptors.rows <<  endl;
				Mat labels(descriptors.rows, 1, CV_32F);
				for (int j = 0; j < descriptors.rows; j++)
				{
					labels.at<float>(j,0) = label;
				}
				// load matrices
				labelMatrix.push_back(labels);
				trainingMatrix.push_back(descriptors);
			}

			cout << endl << "DONE!" << endl << "Total Descriptors : " << trainingMatrix.rows << endl << endl << endl;
			return true;
		}

		/* Loads images in directory in matrix form
			INPUT@ dir -> relative or absolute directory of images
			INPUT@ imageType -> opencv enum descripting how to load image
			OUTPUT@ imgMats -> vector of matrices, each element for each image
		*/
		bool loadImages( char* dir, int imageType, vector<Mat>& imgMats )
		{
			Mat image;
			// directory accessing
			DIR* dp;
			struct dirent* dirp;
			char* tok;
			string path = dir;
			string fileName;

			if ( (dp = opendir(dir)) == NULL)
			{
				cout << "Error opening Directory" << endl;
				return false;
			}
	
			while ( (dirp = readdir(dp)) != NULL)
			{
				if (strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
					continue;
				fileName = dirp->d_name;
				path = dir;
				// parse string using "." as delimeter
				tok = strtok(dirp->d_name, ".");
				tok = strtok(NULL, ".");
				if (strcmp(tok, "jpg") == 0 || strcmp(tok, "JPG") == 0)
				{
					image = imread(path + fileName, imageType);
					if (!image.data){
						cout << "Error Reading image " << fileName << endl;
						continue;
					}
					else
					{
						imgMats.push_back(image);
					}
				}
			}
			closedir(dp);
			cout << "Loaded " << imgMats.size() << " pictures." << endl;
			return true;
		}
		
		bool createTest(char* file, int imageType, Mat& testMatrix)
		{
			Mat img = imread(file, imageType);
			if (!img.data)
			{
				cout << "Error reading test image." << endl;
				return false;
			}
			vector<KeyPoint> keypoints;
			
			_detector->detect(img, keypoints);
			_extractor->compute(img, keypoints, testMatrix);

			cout << "Number of descriptors in test image " << file << " : " << testMatrix.rows << endl;

			return true;
		}
};

		
