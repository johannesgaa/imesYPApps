/************************************************************************
 * @brief      FeatureDetection-class for IMES ButtlerBot Project.
 *
 * 			   - This is a part of the IMES ButtlerBot, a demo exercise
 * 			     for the YouBot
 *
 * @file       shape_matcher.h
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-03-13
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#ifndef SHAPE_MATCHER_H_
#define SHAPE_MATCHER_H_

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

/*ROS Includes*/
#include <ros/ros.h>

/**
 * @class Shape Detector
 */
class shape_matcher {

private:	// Reference parameters

	bool boundingMode;								///< indicator if rect or rotated Rect as bounding element is used
	int camFocalLength;								///< camera focal length
	cv::Mat camera_matrix;							///< camera matrix = Mat(3,3,CV_64FC1,camD);
	cv::Mat distortion_coefficients;				///< distortion coeffitients = Mat(5,1,CV_64FC1,distCoeffD);

	std::vector<cv::Point3d> objP;					///< object points
	cv::Mat objPM;									///< object Points as matrix
	cv::Point roiOFFSET;							///< static offset "fummelsummand"

	std::vector<int> shapeWidthVec;					///< with array of detected objects
	std::vector<int> shapeHeightVec;				///< height array of detected objects


	/**
	 * @brief converts a single channel mat to a three channel mat
	 * @param grayMat input
	 */
	cv::Mat OneCHtoThreeCH(cv::Mat grayMat);

	/**
	 * @brief Detects contours on a binary (b/w) image.
	 * @param image1CH Input Mat - must be 8UC1
	 * @param filtering Set to true to delete all contours that have less than 100 or more than 3000 points.
	 */
	std::vector<std::vector<cv::Point> > contourDetector(cv::Mat image1CH);


	/**
	 * @brief Gets a convex hull around each contour in vector.
	 * @param ContourVec Input vector that containt contours.
	 */
	std::vector<std::vector<cv::Point> > huller(
			std::vector<std::vector<cv::Point> > ContourVec);

	/**
	 * @brief Calculates the X/Z-Distance from the camera to the Object.
	 * @param Contour The input shape to calculate for.
	 * @param image The camera-image. The Functions needs to know the resolution for getting right result. Every image with the same size than the camera image is fine.
	 */
	void calculateDistance(cv::Mat Contour, cv::Mat image);

public:

	int Trackbar1;  							///< minLength in % of img.cols
	int Trackbar2;  							///< gap in % of img.cols
	int Trackbar3;  							///< accuracyInterval in 0.1% of img.cols
	int Trackbar4;  							///< lower threshold ContourFilter
	int Trackbar5;  							///< higher threshold ContourFilter
	int Trackbar6;  							///< compliance
	int Trackbar7;  							///< lower threshold Canny
	int Trackbar8;  							///< higher threshold Canny
	int Trackbar9;  							///< Element Morph
	int Trackbar10; 							///< Element Dilatation
	int Trackbar11; 							///< lower threshold sizeFilter
	int Trackbar12; 							///< higher threshold sizeFilter
	int Trackbar13; 							///< higher threshold sizeFilter

	cv::Point roiMiddle;						///< ROI middle
	cv::Size roiSize;							///< ROI size

	int RatioPro;   							///< maximum distance to ideal aspect ratio in percent

	int LowerThreshold;							///< lower threshold for binary conversion
	int HigherThreshold;						///< upper threshold for binary conversion

	std::vector<std::vector<cv::Point> > ContourVec;		///< array of detected contours
	std::vector<std::vector<cv::Point> > refContourVec;		///< array of reference countours
	std::vector<cv::Point3d> distanceVec;					///< array of detected distances
	cv::Mat refShapeImg;						///< reference shape
	cv::Mat binFrame;							///< binary image
	cv::Mat filteredBinMat;						///< binary image after filtering
	cv::Mat lineFilteredFrame;					///< result image after line filtering
	cv::Mat filterMask;							///< filter mask for line filtering
	cv::Mat colorMask;							///< filter mask for color filtering
	std::string objectName;						///< name of the reference object
	int objectHeight;							///< height of the reference object
	int objectWidth;							///< width of the reference object

	bool showWindows;							///< activator for image windowsa
	bool referenceSet;							///< indicator it reference is processed
	bool aspectRatioFiltering;					///< activator for aspect ratio filtering
	bool aspectDynRatioFiltering;				///< activator for aspect ratio filtering of rotated bounding rects
	bool sizeFiltering;							///< activator for bounding box size filtering
	bool contourFiltering;						///< activator for contour point size filtering
	bool useDistanceCalc;						///< activator for integrated solvePNP distance calculator

	/**
	 * @brief Constructor: Will create the reference shape.
	 * @param refPath Path to the image of the reference shape. The image need to be 8UC1. Provide a b/w one to get optimal results. One the biggest shape will be used. Make sure that there is only one shape on that image!
	 */
	shape_matcher();

	/**
	 * @brief Matching function: This will do the job: Converting the input image, extraction of shapes, compare shapes with reference, keep only those who fit the reference (threshold: 0.1).
	 * @param image Input Mat.
	 */
	void match(cv::Mat image);

	/**
	 * @brief Get-function: Distance to the object.
	 * @param VectorIndex Distances are stored in a vector. This param points to the right one.
	 */
	cv::Point3d getDistance(int VectorIndex);

	/**
	 * @brief Set-function for camera focal length. This is needed for calculation the distaces from camera to object.
	 * @param focalLength Focal length. For logitec Webcam around 550.
	 */
	void setCameraParams(int focalLength, cv::Mat cameraMatrix, cv::Mat distoritionMatrix);

	/**
	 * @brief Converts a BGR image into binary (b/w).
	 * @param imageMat The input Mat.
	 * @param mode Choose between three modes:
	 * 				  default: threshold-filter to convert gray to binay. Threshold: 80
	 * 						1: adaptive_threshold: gaussian-method with blocksize: 5.
	 * 						2: canny-algorithmus. experimental: parameters need to be tuned!
	 * @param equalizing Set to true to equalize the grayscale before converting to binary (b/w).
	 * @param filtering Set the size of the filter-kernel. 0= no filtering, default is 3.
	 * @param floating Set to true to return a 32F Mat (default = false = 8U).
	 */
	cv::Mat BGR2BIN(cv::Mat imageMat, int mode, bool equalizing);


	/**
	 * @brief Color Filter will only keep pixels that are within the color range.
	 * @param image The input Mat.
	 * @param hsv_min Left side of the color range in hsv-format.
	 * @param hsv_max Right side of the color range in hsv-format.
	 * @param maskMode  Set to "1" to return a mask of the image and not the image itself
	 * 					Set to "2" to return a mask that contains a bpundingbox around colored areas.
	 * 					Set to "3" (=default) to return the colored parts on the image itself.
	 */
	cv::Mat colorFilter(cv::Mat image, cv::Scalar hsv_min, cv::Scalar hsv_max,
			int maskMode);

	/**
	 * @brief function for filling reference variables
	 * @param path path to shape image
	 * @param height height of reference object in reality
	 * @param width width of reference object in reality
	 */
	bool setReference(std::string path, int height, int width);

	/**
	 * @brief function for switching between rects or rotated rects as bounding elements
	 * @param mode set the mode (0= rect, 1 = rotated rect)
	 */
	void setBoundingMode(int mode);

	/**
	 * @brief function for searching for unwanted line (such as between floor and wall) and erasing them
	 * @param image input
	 */
	cv::Mat lineFilter(cv::Mat image);

	/**
	 * @brief function for line and/or color filtering
	 * @param image input
	 * @param nuSequency begin a new detection (flush filters)
	 */
	cv::Mat preProcessor(cv::Mat image, bool nuSequency);

	/**
	 * @brief function for flushing filters
	 * @param clear input
	 * @param size size of new filter masks
	 */
	void clearFilter(bool clear, cv::Size size);

	/**
	 * @brief function for binary filtering
	 * @aram input
	 */
	cv::Mat binFilter(cv::Mat image);

};

#endif /* SHAPE_MATCHER_H_ */
