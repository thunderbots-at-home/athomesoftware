// ObjectTrainer.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================


#include <object_recognition_experimental_v1/ObjectTrainer.hpp>
#include <unistd.h>

struct TrainingRow;
string CURRENT_DIRECTORY;
string ObjectTrainer::getClassName(string& directory)
{

	// Returns last directory name
	istringstream iss(directory);
	string token;
	string last;
	while (getline(iss, token, '/'))
	{
		last = token;
	}
	return last;
}


char* ObjectTrainer::getFileType(char* filename)
{
	// Returns the file type of a given filename.
	char* stf = strtok(filename, ".");
	return strtok(NULL, ".");
}

void ObjectTrainer::trainSVM(Mat& trainingMatrix, Mat& trainingLabels) 
{

	ROS_INFO("Training SVM...");

	CvTermCriteria tc = cvTermCriteria( CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 0.000001);
	CvSVMParams param = CvSVMParams();

	param.svm_type = CvSVM::C_SVC;
	param.kernel_type = CvSVM::POLY;

	param.degree = 18; // For poly
	param.gamma = 20; // For poly/rgbf/sigmoid
	param.coef0 = 4;  // For poly/sigmoid

	param.C = 10; // Optimization constant
	param.nu = 0.0;
	param.p = 0.0;

	param.class_weights = NULL;
	param.term_crit.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
	param.term_crit.max_iter = 1000;
	param.term_crit.epsilon = 1e-6;	

	ROS_INFO("Training rows (#Images) %d Training cols (#Features)%d", trainingMatrix.rows, trainingMatrix.cols);
	ROS_INFO("Labelled rows (#Images) %d Labelled cols %d", trainingLabels.rows, trainingLabels.cols);

	this->svm.train_auto( trainingMatrix, trainingLabels, Mat(), Mat(), param, 10 );
	this->svm.save( SVM_SAVE_NAME.c_str(), 0);
	ROS_INFO("Trained SVM!");
	ROS_INFO("Saving SVM as %s in your current directory.", SVM_SAVE_NAME.c_str());
	
}

void ObjectTrainer::normalize(string image_file, Mat& output_image)
{
	Mat image;

	image = imread( image_file.c_str(), CV_8UC1);
	if (!image.data) { ROS_INFO("Error reading Image %s", image_file.c_str()); return ;}
	// 3. DoG bandpass filter to avoid aliasing.
	Mat dog_mat;
	Mat dog1, dog2;
	GaussianBlur(image, dog1, Size(11, 11), 0);
	GaussianBlur(image, dog2, Size(51, 51), 0);
	dog_mat = (dog1 - dog2);
	// 4. Normalize image to standard resolution
	resize(dog_mat, output_image, Size(256, 256)); 
	assert(output_image.type() == CV_8U);
}

void ObjectTrainer::computeTrainingMatrix(Mat& trainingMatrix, Mat& trainingLabels, Mat& vocabulary)
{
	
	ROS_INFO("Computing training matrix");
	BOWImgDescriptorExtractor bowide(this->extractor, this->matcher);
	assert(vocabulary.type() == CV_32F);
	bowide.setVocabulary(vocabulary);

	int divider = training_rows.size() - training_rows.size()*0.3;
	// Select 30% of training rows randomly, remove them from the vector, and put them into a testing row
	int test_size = training_rows.size()-divider;	

	ROS_INFO("Using %d out of %d images for training (70 percent)", divider, training_rows.size());
	ROS_INFO("Randomly selecting %d images from the training set for testing", test_size);
	
	vector<TrainingRow*> testing_rows;
	
		for (int p = 0; p < test_size; p++)
		{
			int rand_val = rand() % training_rows.size();
			testing_rows.push_back(training_rows.at(rand_val));
			training_rows.erase(training_rows.begin()+rand_val);
		}
		
		assert(testing_rows.size() == test_size);

		ROS_INFO("Creating training matrix");
		for (int i = 0; i < divider; i++)
		{

			Mat normalizedMat;
			Mat descriptors;	
			Mat response_histogram;
			response_histogram.convertTo(response_histogram, CV_32F);
			vector<KeyPoint> keypoints;

			const char* imageFilename = training_rows.at(i)->image_name.c_str();

			normalize(imageFilename, normalizedMat);
			detector->detect(normalizedMat, keypoints);
			bowide.compute(normalizedMat, keypoints, response_histogram);
			if (response_histogram.data)
			{
				response_histogram.convertTo(response_histogram, CV_32F);
				assert( response_histogram.type() == CV_32F);
				trainingMatrix.push_back(response_histogram);
				trainingLabels.push_back(training_rows.at(i)->class_label);
			}
		}
		ROS_INFO("Training matrix generated");
		ROS_INFO("Creating testing matrix");
		for (int k = 0; k < testing_rows.size(); k++)
		{
			// Add this to the test data set.
			Mat normalizedMat;
			Mat descriptors;	
			Mat response_histogram;
			response_histogram.convertTo(response_histogram, CV_32F);
			vector<KeyPoint> keypoints;

			const char* imageFilename = testing_rows.at(k)->image_name.c_str();

			normalize(imageFilename, normalizedMat);
			detector->detect(normalizedMat, keypoints);
			bowide.compute(normalizedMat, keypoints, response_histogram);

			if (response_histogram.data)
			{
				response_histogram.convertTo(response_histogram, CV_32F);
				assert( response_histogram.type() == CV_32F);
				this->testingMatrix.push_back(response_histogram);
				this->testingLabels.push_back(testing_rows.at(k)->class_label);
			}


		}
	ROS_INFO("Testing matrix generated");
	
}

void ObjectTrainer::computeVocabulary(Mat& output_vocabulary)
{

	ROS_INFO("Computing vocabulary");	
	char ** argv = directories;

	int CLASSLABEL = 1;
	std::string current;
	TermCriteria TC( CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 0.001 );
	BOWKMeansTrainer bow ( KMEANS_CLUSTERS, TC, KMEANS_ATTEMPTS, KMEANS_PP_CENTERS);

	ROS_INFO("Number of classes detected: %d", NUM_CLASSES-1);
	
	while (CLASSLABEL < (NUM_CLASSES))
	{
		current = argv[CLASSLABEL];
		ROS_INFO("%s", current.c_str());
		DIR *pDIR;
		struct dirent *entry;	
		pDIR = opendir(argv[CLASSLABEL]);
		string label;
		label = getClassName(current);
		current.append("/");
	

		if (pDIR)
		{

			ROS_INFO("Processing images in directory %s", current.c_str());
			while ((pDIR != NULL ) && (entry = readdir(pDIR)))
			{
				if ( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
				{
					char* filetype = getFileType(entry->d_name);
					if ((filetype != NULL) && strcmp(filetype, "jpg") == 0)
					{
						NUM_IMAGES++;
						std_msgs::String imageData;
						std::string copy = current;
						imageData.data = (copy.append(entry->d_name))+".jpg";

						TrainingRow* row = new TrainingRow;
						row->image_name = imageData.data;
						row->class_label = CLASSLABEL;

						training_rows.push_back(row);

						Mat descriptors;
						Mat normalizedMat;
						normalize(imageData.data, normalizedMat);
						vector<KeyPoint> keypoints;
						detector->detect(normalizedMat, keypoints);
						extractor->compute(normalizedMat, keypoints, descriptors);
						for (int i = 0; i < descriptors.rows; i++)
						{
							if (!descriptors.empty()) 
							{ 
								Mat m = descriptors.row(i);
								m.convertTo(m, CV_32F);
								bow.add(m); 	
							}
						}
					}	
				}
				

			}
			closedir(pDIR);
		} 
		else 
		{
			ROS_INFO("Invalid directory name %s", current.c_str());
		}

		CLASSLABEL++;

	}
	ROS_INFO("Descriptors: %d", bow.descripotorsCount());
	if (bow.descripotorsCount() == 0) 
	{ 
	ROS_INFO("No images computed. Please check that you have entered valid directory names and that images are of type .jpg"); 
	}
	output_vocabulary = bow.cluster();
	FileStorage fs("vocabulary.yml", FileStorage::WRITE);
	fs << "vocabulary" << output_vocabulary;

	ROS_INFO("Model of data created");
}

void ObjectTrainer::computeHistogram(string image_file, Mat& vocabulary, Mat& response_histogram)
{
	ROS_INFO("Computing histogram");

	BOWImgDescriptorExtractor bowide(this->extractor, this->matcher);
	bowide.setVocabulary(vocabulary);

	Mat normalized_image;
	normalize(image_file, normalized_image);			
	
	vector<KeyPoint> keypoints;
	detector->detect(normalized_image, keypoints);	
	
	bowide.compute(normalized_image, keypoints, response_histogram);
	ROS_INFO("Computed histogram");

}

float ObjectTrainer::testSVM()
{
	ROS_INFO("Testing SVM implementation.");


	CvSVM svm;
	svm.load(SVM_SAVE_NAME.c_str());
	

	float num_test_images = testingMatrix.rows;
	float correct;
	float incorrect;

	for (int m = 0; m < (int)num_test_images; m++)
	{
		int prediction = svm.predict(testingMatrix.row(m));
		if (prediction == testingLabels.at<int>(m, 0)) { correct++; } else { incorrect++; }
	}

	ROS_INFO("Correct classifications %f", correct);
	ROS_INFO("Incorrect classifications %f", incorrect);

	return (float)(correct/num_test_images);
}


ObjectTrainer::~ObjectTrainer() 
{
	for (int k = 0; k < training_rows.size(); k++) { delete training_rows.at(k); }
}

ObjectTrainer::ObjectTrainer()
: SVM_SAVE_NAME("TrainedSVM"), VOCABULARY_SAVE_NAME("vocabulary"), KMEANS_ATTEMPTS(3), KMEANS_CLUSTERS(500), NUM_IMAGES(0) 
{

}

ObjectTrainer::ObjectTrainer(int argc, char** argv, Ptr<FeatureDetector>& fdetector, Ptr<DescriptorExtractor>& dextractor, 			Ptr<DescriptorMatcher>& dmatcher, int clusters, int attempts) 
: KMEANS_ATTEMPTS(3), KMEANS_CLUSTERS(186), NUM_IMAGES(0), SVM_SAVE_NAME("TrainedSVM"), VOCABULARY_SAVE_NAME("vocabulary")
{
	SVM_SAVE_NAME = CURRENT_DIRECTORY + "/config/TrainedSVM";
	VOCABULARY_SAVE_NAME = CURRENT_DIRECTORY + "/config/vocabulary.yml";
	ROS_INFO("Saving SVM to %s", SVM_SAVE_NAME.c_str());
	ROS_INFO("Saving Vocabulary to %s", VOCABULARY_SAVE_NAME.c_str());

	KMEANS_CLUSTERS = clusters;	
	KMEANS_ATTEMPTS = attempts;
	this->extractor = dextractor;
	this->matcher = dmatcher;
	this->detector = fdetector;
	NUM_CLASSES = argc;

	directories = argv;

	ROS_INFO("Computing vocab..");
	this->computeVocabulary(vocabulary);
	this->computeTrainingMatrix(trainingMatrix, trainingLabels, vocabulary);
	this->trainSVM(trainingMatrix, trainingLabels);

	float accuracy_pct = this->testSVM();
	ROS_INFO("Accuracy of SVM %f percent", accuracy_pct);
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "object_trainer");
	ros::NodeHandle n;

	//Finding the current directory
	char buf[PATH_MAX];
	string dir(getwd(buf));
	CURRENT_DIRECTORY = dir;
	ROS_INFO("Current dir %s", CURRENT_DIRECTORY.c_str());

	ROS_INFO("Creating vocabulary.yml and TrainedSVM.txt!");

	Ptr<FeatureDetector> detector(new SurfFeatureDetector( 400 ));
	Ptr<DescriptorExtractor> extractor(new SurfDescriptorExtractor);
	Ptr<DescriptorMatcher> matcher(new BruteForceMatcher<L2<float> >());

	ObjectTrainer trainer(argc, argv, detector, extractor, matcher, 186, 3);
	ROS_INFO("Successfully created vocabulary.yml and TrainedSVM.txt!");

	return 0;
}

