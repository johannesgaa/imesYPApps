/************************************************************************
 * @brief      FeatureDetection-class for demonstration purposes
 *
 * 			   - This is a part of imes_vision and imes_butlerBot and
 * 			     may not run in other surroundings
 *
 *
 * @file       RC_feature.h
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-06-23 / 2013-02-12
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#ifndef RC_FEATURE_H_
#define RC_FEATURE_H_

/* c/c++ Includes */
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <limits>
#include <fstream>
#include <string>

#define _USE_MATH_DEFINES
#include <cmath>

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>

/* Detector Class */
#include <imes_vision/ObjectFinder.h>

/* Service */
#include <imes_vision/FeatureDetectorReference.h>

/**
 * @brief Demonstration class for Feature Detection
 */
class rc_features {

public:

	/**
	 * @brief Class constructor
	 * @param cameraTopic name of image source topic
	 * @param showCVwindows (de)-activates graphical output
	 */
	rc_features(std::string cameraTopic, bool showCVwindows);

	/**
	 * @brief Image callback method
	 * @param msg picture in sensor_msg format
	 */
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);


	/**
	 * @brief Main method
	 */
	void rc_feature_main();

	/**
	 * @brief Displaying method
	 */
	void displayer();

	/**
	 * @brief method to change object to find
	 */
	void changeReference();

	/**
	 * @brief method to change object to find
	 * @param logo number (1=imes, 2=tq)
	 */
	void changeReference(int logo);

	/**
	 * @brief service method
	 * @param req Service Request
	 * @param resp Service Response
	 */
	bool Service(imes_vision::FeatureDetectorReference::Request &req,
			imes_vision::FeatureDetectorReference::Response &res);

	/**
	 * @brief structure that holds basic data of an detectable object
	 */
	struct testObject {
		 std::string featurePath;					///< image path
		 int numOfPics;								///< number of source ictures to load from path
		 std::string shapePath;						///< shape path
		 int shapeHeight;							///< original height of object in mm
		 int shapeWidth;							///< original width of object in mm
	 } ;

private:

	ros::NodeHandle nh;								///< ros node handle
	ros::ServiceServer rev_service;					///< ros service for changing the object reference

	image_transport::Subscriber image_sub;			///< ros image subscriber
	image_transport::ImageTransport it;				///< ros image transporter
	image_transport::Publisher image_pub;			///< ros image publisher

	feature_matcher my_feature_matcher;				///< feature matching class

	boost::mutex cb_mutex;							///< mutex for locking the image callback

	cv::Mat origFrame; 								///< original frame from camera

	bool showWindows;								///< (de)-activates graphical output
	bool match;										///< indicated if object is detected

	int minFeatures;								///< minimal Features to recognize the object

	testObject tstObj;								///< object to detect
	testObject imes_logo;							///< object 1
	testObject tq_logo;								///< object 2

	cv_bridge::CvImage roscv_outImage;				///< data format for reublishing image

	std::vector<cv::DMatch> matches;				///< descriptor matches
	std::vector<cv::KeyPoint> keypoints1;			///< keypoints of live image
	std::vector<cv::KeyPoint> keypoints2;			///< keypoints of reference image

	cv::Rect roiBox;								///< rectangle around area to detect
	cv::Mat stickerROI;								///< image containing the area to detect

	feature_matcher::matchInfo matchInfo;			///< infomation about last matching process;
};
#endif /* RC_FEATURE_H_ */
