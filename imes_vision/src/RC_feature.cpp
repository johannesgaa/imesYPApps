/************************************************************************
 * @brief      FeatureDetection-class for demonstration purposes
 *
 * 			   - This is a part of imes_vision and imes_butlerBot and
 * 			     may not run in other surroundings
 *
 *
 * @file       RC_feature.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-06-23 / 2013-02-12
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <imes_vision/RC_feature.h>


rc_features::rc_features(std::string cameraTopic, bool showCVwindows) : it(nh), my_feature_matcher() {


	/* Say hello */
	ROS_INFO("Object Finder initializing");
	ROS_INFO("looking for picture stream from webcam..");


	// Objects
	// getting path of product files:
	std::string fPath, tmp;
	fPath= ros::package::getPath("imes_butlerBot");
	tmp= fPath;

	testObject Becks;
	testObject Sol;
	testObject Indio;
	testObject XX;

	// initialize some Objects:
	if (true) {
		tmp= fPath;
		tmp.append("/resources/products/Sol");
		Sol.featurePath = tmp;
		Sol.numOfPics = 6;
		tmp= fPath;
		tmp.append("/resources/shapes/std_beer_bottle_033.png");
		Sol.shapePath = tmp;
		Sol.shapeHeight = 235;
		Sol.shapeWidth = 60;

		tmp= fPath;
		tmp.append("/resources/products/Indio");
		Indio.featurePath = tmp;
		Indio.numOfPics = 5;
		tmp= fPath;
		tmp.append("/resources/shapes/std_beer_bottle_033.png");
		Indio.shapePath = tmp;
		Indio.shapeHeight = 235;
		Indio.shapeWidth = 60;

		tmp= fPath;
		tmp.append("/resources/products/XX");
		XX.featurePath = tmp;
		XX.numOfPics = 5;
		tmp= fPath;
		tmp.append("/resources/shapes/std_beer_bottle_033.png");
		XX.shapePath = tmp;
		XX.shapeHeight = 235;
		XX.shapeWidth = 60;

		tmp= fPath;
		tmp.append("/resources/products/Becks_Pils_033");
		Becks.featurePath = tmp;
		Becks.numOfPics = 8;
		tmp= fPath;
		tmp.append("/resources/shapes/std_beer_bottle_033.png");
		Becks.shapePath = tmp;
		Becks.shapeHeight = 235;
		Becks.shapeWidth = 60;
	}

	// Default object to find
	tstObj= Becks;
	match = false;

	if (showCVwindows == true) {
		ROS_INFO("ObjectFinder will show image-windows.");
		this->showWindows = true;
		my_feature_matcher.showWindows = false;
		cv::namedWindow("Cam_Stream", CV_WINDOW_AUTOSIZE);
		cv::startWindowThread();
	} else {
		ROS_WARN("ObjectFinder will not show any image-windows!");
		this->showWindows=false;
		my_feature_matcher.showWindows=false;
	}

	/* Registering callback and service */
	this->image_sub  = it.subscribe(cameraTopic,   1, &rc_features::imageCallback, this);

	/* setting feature matcher settings */
	my_feature_matcher.setRobustness(2, 7);
	my_feature_matcher.setReference(tstObj.featurePath, tstObj.numOfPics);

	/* INIT DONE */
	ROS_INFO("Object Finder Service started!");
}


void rc_features::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cb_mutex.lock();
	cv_bridge::CvImagePtr cv_ptr;

	try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        this->origFrame = cv_ptr->image;
        rc_feature_main();
        displayer();
	} catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
	}
	cb_mutex.unlock();
}

void rc_features::changeReference() {

	my_feature_matcher.setReference(tstObj.featurePath, tstObj.numOfPics);
}

void rc_features::rc_feature_main() {
	// get origFrame
	while (image_sub.getNumPublishers() == 0) {
		ROS_INFO("Waiting for image topic..\n");
		ros::Duration(2).sleep();
	}

	// define ROI around gripper_space
	roiBox.x=(int)(0.40 * this->origFrame.size().width);
	roiBox.y=(int)(0.25 * this->origFrame.size().height);
	roiBox.width=(int)(0.20 * this->origFrame.size().width);
	roiBox.height=(int)(0.5 * this->origFrame.size().height);

	// save ROI for detection in new Mat
	stickerROI = this->origFrame(roiBox);

	// preparing image:
	cv::Mat output(this->origFrame.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	std::stringstream onScreenText;
	std::vector<cv::Rect> stickerAry;
	stickerAry.clear();

	// Feature Matching: Match live image and reference
	cv::Mat dummy;
	matches.clear();
	keypoints1.clear();
	keypoints2.clear();
	match = my_feature_matcher.match(stickerROI, dummy, matches, keypoints1, keypoints2);
}

void rc_features::displayer() {

	cv::Mat flipped;
	cv::Size outSize;

	// calculating the window size: height
	if ((this->origFrame.rows) >= (my_feature_matcher.allReferences.cols))
		outSize.height= this->origFrame.rows;
	else
		outSize.height= my_feature_matcher.allReferences.size().width;
	// calculating the window size: width
	outSize.width= (int)(this->origFrame.cols + my_feature_matcher.allReferences.rows);

	cv::Mat output(outSize, CV_8UC3, cv::Scalar(128,0,0));
	cv::Mat roi2 = output(cv::Rect(this->origFrame.size().width, 0,   my_feature_matcher.allReferences.size().height,   my_feature_matcher.allReferences.size().width));
	roi2.setTo(cv::Scalar(255,255,255));

	// flip  "allRefs" 90°
	cv::transpose(my_feature_matcher.allReferences, flipped);
	cv::flip(flipped, flipped, 1);

	// draw keypoints if positive match
	if (match) {
		ROS_INFO("OBJECT FOUND!");
		cv::rectangle(this->origFrame, roiBox, cv::Scalar(0,200,0), 2, 8);
	} else {
		ROS_INFO("OBJECT NOT FOUND");
		cv::rectangle(this->origFrame, roiBox, cv::Scalar(0,0,200), 2, 8);
	}

	// Print Framerate and Keypoints on screen
	matchInfo = my_feature_matcher.info_lastMatch;
	std::stringstream onScreenText;
	onScreenText.str(std::string());
	onScreenText << "FPS: " << 1000./matchInfo.processingTime << "Hz";
	cv::putText(this->origFrame, onScreenText.str(), cv::Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 10, 10), 1, CV_AA);
	onScreenText.str(std::string());
	onScreenText << "Computing Time: " << matchInfo.processingTime << "ms";
	cv::putText(this->origFrame, onScreenText.str(), cv::Point(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 10, 10), 1, CV_AA);
	onScreenText.str(std::string());
	onScreenText << "Processed keypoints: " << (matchInfo.imageKeys + matchInfo.referenceKeys);
	cv::putText(this->origFrame, onScreenText.str(), cv::Point(10, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 10, 10), 1, CV_AA);

	// output = 2 ROIS (origFrame + flip)
	cv::Mat roi1 = output(cv::Rect(0,               0,                this->origFrame.size().width, this->origFrame.size().height));
	this->origFrame.copyTo(roi1);
	flipped.copyTo(roi2);
	cv::imshow("Cam_Stream", output);
}

/**
 * @brief main function
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "IMES_Object_Finder");

	std::string cameraTopic;
	bool showCVwindows;

	// checking arguments:
	if (argc == 2) {
		if ((strcmp(argv[1], "-simulator") == 0) || (strcmp(argv[1], "-gazebo") == 0)) {
			cameraTopic = "/wrist_cam_front/image_raw";
		} else {
			cameraTopic = "/usb_cam/image_raw";
		}
		if ((strcmp(argv[1], "-no window") == 0) || (strcmp(argv[1], "-youbot") == 0)) {
			showCVwindows = false;
		} else {
			showCVwindows = true;
		}
	} else {
		cameraTopic = "/usb_cam/image_raw"; // assuming bosch usb cam node is used!
		showCVwindows= true;
	}
	std::cout << "Set camera-topic to " << cameraTopic << std::endl;

	// run detector
	rc_features  my_FeatureFinder(cameraTopic, showCVwindows);

	ros::spin();
	return 0;
}


