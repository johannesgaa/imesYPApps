/************************************************************************
 * @brief      FeatureDetection-class for IMES ButtlerBot Project.
 *
 * 			   - This is a part of the IMES ButtlerBot, a demo exercise
 * 			     for the YouBot
 *
 * @file       feature_matcher.h
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-03-13 modified 2012-05-08
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#ifndef FEATURE_MATCHER_H_
#define FEATURE_MATCHER_H_

/* c/c++ Includes */
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <limits>
#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

/* OpenCV Includes */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>

#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/features2d.hpp>

/* ROS includes */
#include <ros/ros.h>

	/**
 * @class feature_Matcher
 */
class feature_matcher {

private:

	std::string refName;						///< name of reference object
	int numOfRefPics;							///< number on reference pictures

	// For setRobustness
	bool rTest12;								///< activator for image->reference matching
	bool rTest21;								///< activator for reference->image matching
	bool symTest;								///< activator of matching symmetry checking
	bool rsacTest;								///< activator for ransac test

	int Trackbar1;								///< minLength in % of img.cols
	int Trackbar2; 								///< gap in % of img.cols
	int Trackbar3; 								///< accuracyInterval in 0.1% of img.cols
	int Trackbar4;

	int minimumMatches;							///< minimum positive matches needed to detect an object
	long totalreferenceKeys;						///< number of keypoints in all referenze images


	/**
	 * @brief This filters the feature-detector-results. All matches that are too far will be deleted
	 * @param matches The matches created by BruteForceMatcher
	 */
	int ratioTest(std::vector<std::vector<cv::DMatch> >& matches);

	/**
	 * @brief This is filtering. It looks if the nearest neigbour matches of Pic1 and Pic2 are equal. Others will be deleted
	 * @param matches1 Matches from Pic1
	 * @param matches2 Matches from Pic2
	 * @param symMatches This is the output variable!
	 */
	void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
			const std::vector<std::vector<cv::DMatch> >& matches2,
			std::vector<cv::DMatch>& symMatches);

	/**
	 * @brief This will try to create a fundemental matrix. Search the web for info!
	 * @param matches The feature matches
	 * @param keypoints1 Keypoints from Pic1
	 * @param keypoints2 Keypoints from Pic2
	 * @param outMatches Output variable
	 */
	cv::Mat ransacTest(const std::vector<cv::DMatch>& matches,
			const std::vector<cv::KeyPoint>& keypoints1,
			const std::vector<cv::KeyPoint>& keypoints2,
			std::vector<cv::DMatch>& outMatches);

public:


	cv::Ptr<cv::FeatureDetector> detector;						///< pointer to the feature point detector object
	cv::Ptr<cv::DescriptorExtractor> extractor;					///< pointer to the feature descriptor extractor object

	// SIFT parameters:
	float ratio; 												///< max ratio between 1st and 2nd NN
	bool refineF; 												///< if true will refine the F matrix
	double confidence;											///< confidence level (probability)
	double distance;											///< min distance to epipolar

	float highestComp;											///< highest compliance

	bool referenceSet;											///< indicator that reference pictures are processed correctly
	bool showWindows;											///< activator for image windows

	// reference variables
	cv::Mat refImg;												///< reference image
	cv::Mat allReferences;										///< all references on one image
	cv::Mat refGrayImg;											///< reference as one ch mat
	std::vector<std::vector<cv::KeyPoint> > refKeypoints;		///< referece Keypoints
	std::vector<cv::Mat> refDescriptors;						///< reference descriptors
	std::vector<cv::Mat> referencePics;							///< array of reference images

	/**
	 * @brief structure to provide relevant matching information
	 */
	struct matchInfo {
		int processingTime;
		int referenceKeys;
		int imageKeys;
		int positiveMatches1;
		int positiveMatches2;
	} ;
	matchInfo info_lastMatch;


	/**
	 * @brief class constructor
	 */
	feature_matcher();

	/**
	 * @brief Set robustness of matcher (1 = minumum <-> 4 = maximum):
	 * @param robustness This controlles the filtering (the robustness):
	 * 1: (default:) only nearest-neighbor-ratio-test from image with reference
	 * 2: equal to 1, but additionally nearest-neighbor-ratio-test from reference with iamge (the other way compared to 1)
	 * 3: equal to 2, additionally symmetry-Test
	 * 4: equal to 3, additionally ransacTest
	 * @param neededPositiveMatches minimum matches to identify an object
	 */
	void setRobustness(int robustness);
	void setRobustness(int robustness, int neededPositiveMatches);


	/**
	 * @brief This will read and prepare the reference pictures
	 * @param path The Folder with the pics
	 * @param int numPics The number of pics that shall be read
	 */
	void setReference(std::string path, int numPics);

	/**
		 * @brief This is doing the detection
		 * @param detect Pointer to detector
		 */
	void setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect);

	/**
		 * @brief With this you can set description-extractor
		 * @param desc Pointer to description-extractor
		 */
	void setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc);

	/**
	 * @brief Setter for Matcher
	 * @param d Distance
	 */
	void setMinDistanceToEpipolar(double d);

	/**
	 * @brief Setter for Matcher
	 * @param d Distance
	 */
	void setConfidenceLevel(double c);

	/**
	 * @brief Setter for Matcher
	 * @param r Ratio
	 */
	void setRatio(float r);

	/**
	 * @brief Setter for Matcher
	 * @param flag Flag
	 */
	void refineFundamental(bool flag);

	/**
	 * @brief This is doin' the matching
	 * @param image1 Pic1 (reference)
	 * * @param image2 Pic2 (from cam)
	 * * @param matches The matches of both
	 * * @param keypoints1 Keypoints from Pic1
	 * * @param keypoints2 Keypoints from Pic2
	 */
	bool match(cv::Mat& image1,
			cv::Mat& image2, // input images
			std::vector<cv::DMatch>& matches, // output matches and keypoints
			std::vector<cv::KeyPoint>& keypoints1,
			std::vector<cv::KeyPoint>& keypoints2);
};


#endif /* FEATURE_MATCHER_H_ */
