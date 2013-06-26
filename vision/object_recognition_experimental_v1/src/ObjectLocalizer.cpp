// ObjectRecognizer.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================

#include <object_recognition_experimental_v1/ObjectLocalizer.hpp>
ObjectLocalizer::~ObjectLocalizer()
{

}

ObjectLocalizer::ObjectLocalizer(Mat& mImage, Mat& object_pic, Mat& output)
{
	SurfFeatureDetector detector(400);
	SurfDescriptorExtractor extractor;
	FlannBasedMatcher matcher;

	vector<KeyPoint> keypoints_picture, keypoints_scene;
	
	detector.detect(object_pic, keypoints_picture);
	detector.detect(mImage, keypoints_scene);

	Mat descriptors_picture, descriptors_scene;			

	extractor.compute(object_pic, keypoints_picture, descriptors_picture);
	extractor.compute(mImage, keypoints_scene, descriptors_scene);

	vector<DMatch> matches;

	matcher.match(descriptors_picture, descriptors_scene, matches);
	
	double min_dist = 100;
	double max_dist = 0;

	for (int i = 0; i < descriptors_picture.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) { min_dist = dist; }
		if (dist > max_dist) { max_dist = dist; }
	}	

	vector<DMatch> good_matches;

	for (int i = 0; i < descriptors_picture.rows; i++)
	{
		if (matches[i].distance < 3*min_dist)
		{
			good_matches.push_back(matches[i]);
		}
	}
		
	Mat img_matches;

	drawMatches(object_pic, keypoints_picture, mImage, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	// Localize the object
	vector<Point2f> obj;
	vector<Point2f> scene;

	for (int i = 0; i < good_matches.size(); i++)
	{
		obj.push_back(keypoints_picture[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}	

	Mat H = findHomography(obj, scene, CV_RANSAC);

	// Get the corners from the image_1
	vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(object_pic.cols, 0);
	obj_corners[2] = cvPoint(object_pic.cols, object_pic.rows); obj_corners[3] = cvPoint(0, object_pic.rows);
	vector<Point2f> scene_corners(4);
	perspectiveTransform(obj_corners, scene_corners, H);

	// Draw lines beteween the corners
	line(img_matches, scene_corners[0] + Point2f(object_pic.cols, 0), scene_corners[1] + Point2f(object_pic.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1] + Point2f(object_pic.cols, 0), scene_corners[2] + Point2f(object_pic.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2] + Point2f(object_pic.cols, 0), scene_corners[3] + Point2f(object_pic.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3] + Point2f(object_pic.cols, 0), scene_corners[0] + Point2f(object_pic.cols, 0), Scalar(0, 255, 0), 4);

	output = img_matches;
		
}
