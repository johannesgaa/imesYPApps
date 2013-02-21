/************************************************************************
 * @brief      ObjectFinder-class for IMES ButtlerBot Project.
 *
 * 			   - This is designed for IMES ButtlerBot, a demo exercise
 * 			     for the YouBot. This class controles the two-step-pipeline
 * 			     for identifying objects with a 2D webcam.
 *
 * @file       ObjectFinder.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-03-13
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

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
#include <opencv/cv.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <boost/thread.hpp>
#include <imes_vision/object_finder.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

/* Namespaces */
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

/* Server include */
#include <imes_vision/ObjectFinderService.h>

/* Detector Classes */
#include <imes_vision/feature_matcher.h>
#include <imes_vision/shape_matcher.h>

/**
 * @brief Finder class for managing the detection of shapes and features in a 2D webcam image
 */
class ObjectFinder {

public:

	/**
	 * brief structure for object reference
	 */
	struct objectDescription {
		std::string name;
		std::string shapePath;
		std::string featurePath;
		int numberOfPictures;
		int height;
		int width;
	} wantedProduct;

	/**
	 * @brief structure for result of feature detection
	 */
	struct productDescription {
		string productName;
		float featureCompliance;
		int positiveShapeResults;
		std::vector<cv::Point3d> position;
	};

	// CLASSES
	shape_matcher my_shape_matcher; 			///< shape detector class
	feature_matcher my_feature_matcher; 		///< feature detector class
	tf::TransformListener listener; 			///< tf listener for transfomation calculations

	/// variables
	bool camIsWorking; 							///< indicator that camera topic is received
	bool showWindows; 							///< indicator that graphical windows can be shown
	bool permanentShapeDetection; 				///< indicator for callback that shape detection is permanent (and not only when service requested)
	bool calibrationDone; 						///< indicator that cameraInfoDopic is processed and image callback ready

	cv::Mat origFrame; 							///< original frame from camera
	cv::Mat distoredFrame; 						///< undistored camera frame
	cv::Size imageSize; 						///< image size (width x height)
	cv::Mat cameraMatrix; 						///< camera matrix
	cv::Mat distCoeffs; 						///< distortion coefficients
	cv::Mat frameWithStatus; 					///< frame with statusbar
	boost::mutex cb_mutex;						///< mutex for locking/unlocking original image
	int sequencyLength; 						///< number of frames for sequency filtering and detection
	ros::Rate* loopRate; 						///< maximum rate for imageCallbacks


	/**
	 * @brief Constructor. This initiallizes to shape_matcher and feature_matcher -objects
	 * @param shape_path The path to a reference shape image. The referece must be b/w and stored as grayscale.
	 * @param feature_path The path to a reference image to extract features from.
	 */
	ObjectFinder(std::string cameraTopic, std::string cameraInfoTopic,
			std::string cameraLink, std::string mode, bool showCVWindows);

	/**
	 * @brief Destructor
	 */
	virtual ~ObjectFinder() {
	}
	;

	/**
	 * @brief imageCallback when two stage detection as service is wanted
	 */
	void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);

	/**
	 * @brief This function will be called when a new message (image) is received. This Callback is doin a feature detection when the service is called
	 */
	void imageCallback_feature_service(const sensor_msgs::ImageConstPtr& msg);

	/**
	 * @brief imageCallback for shape detection service.
	 */
	void imageCallback_shape_service(const sensor_msgs::ImageConstPtr& msg);

	/**
	 * @brief imageCallback for permanent shape detection
	 */
	void imageCallback_shape_topic(const sensor_msgs::ImageConstPtr& msg);

	/**
	 * @brief this can be used if only a part on the image should be active for detection
	 */
	bool setROI(cv::Point roiMiddle, cv::Size roiSize);

	/**
	 * @brief Service function. Called when client wants feature detection
	 * @param req Request structure sent by client
	 * @param res Response structure will be sent to client when service is finished
	 */
	bool Feature_Detector_Service(
			imes_vision::ObjectFinderService::Request &req,
			imes_vision::ObjectFinderService::Response &res);

	/**
	 * @brief Service function. Called when client wants shape detection
	 * @param req Request structure sent y client
	 * @param res Response structure will be sent when service finished
	 */
	bool Shape_Detector_Service(imes_vision::ObjectFinderService::Request &req,
			imes_vision::ObjectFinderService::Response &res);

	/**
	 * @brief fction for publishing on imes_vision BottleDetector-topic
	 */
	void publishMessage();

	/**
	 * @brief Funtion that controles the line filtering
	 * @param input image
	 * @param sequencyLength number of frames that will be analysed for a single filtering stage
	 */
	void PreProcessor(cv::Mat input, int sequencyLength);

	/**
	 * @brief shape detection function
	 * @param input contour that will be matched
	 */
	std::vector<std::vector<cv::Point> > ShapeProcessor(cv::Mat input);

	/**
	 * @brief Function that can collect results of a number of frames. Each result will be checked if it's already known. Smaller shapes will correct wider ones
	 * @param input array of boundingboxes around positive matched shapes
	 * @param sequencyLength number of frames that will be analysed as one sequence
	 */
	void PostProcessor(std::vector<std::vector<cv::Point> > input,
			int sequencyLength);

	/**
	 * @brief fetaure detection function
	 * @param input input image
	 */
	void FeatureProcessor(std::vector<std::vector<cv::Point> > input);

	/**
	 * @brief display function
	 * @param total window size
	 */
	void Displayer(cv::Size inputSize);


private:

	string CAMERALINK;										///< constant: frame_id of camera
	string MODE;											///< detection mode

	ros::NodeHandle nh;										///< node handle
	ros::Subscriber camerainfo_sub;							///< cameraInfo subscriber
	image_transport::Subscriber image_sub;					///< image subscriber
	image_transport::ImageTransport it;						///< image transport class

	// ROS Publisher & Service
	ros::ServiceServer my_service;							///< instance of serviceServer
	ros::Publisher object_finder_publisher;					///< publisher for imes_vision/BottleDetection

	bool cmdWaiting;										///< blocker for sinle cmd output ("waiting for clients)

	bool lineFiltering;										///< activator for line filtering
	bool colorFiltering;									///< activator for color filtering

	bool serviceCalled;										///< indicator that client called service (will unblock detection functions in callback)
	bool detectionInProgress;								///< indicator that a service is running and not finished
	bool nuSequency;										///< indicator that a new sequence starts (deletes all previous results)
	bool preProcessorRdy;									///< indicator that preprocessing is done (and shape detection can start)bool lineFiltering;
	bool postProcessorRdy;									///< indicator that postprocessing is done (and result can be sent to service/topic)
	bool stage1complete;									///< indicator that shape detection is done (and on two step pipeline feature detection can start)
	int imagePipeline;										///< indicator how to configure two stage pipeline

	int postFrameCounter;									///< counter for postprocessor
	int filterFrameCounter;									///< counter for preprocessor

	productDescription actualProduct;						///< reference object

	cv::Mat filteredFrame;									///< filtered frame
	cv::Point roiMiddle;									///< middle of ROI
	cv::Size roiSize;										///< size of ROI

	std::vector<cv::Rect> stickerArray;						///< sticker of positive mathed shapes
	std::vector<cv::Rect> posStickerArray;					///< positive detected stickers
	std::vector<std::vector<cv::Point> > ContourVec0;		///< contour array of first postprocessor frame
	std::vector<std::vector<cv::Point> > postProcessedArray;///< result array (postprocessor finished)
	std::vector<std::vector<cv::Point> > messageArray;		///< this array will be published on topic
	std::vector<cv::Rect> CV0Boxes;							///< array of boundingboxes around shapes of first postprocessor frame
	std::vector<cv::Point3d> distToShapes;					///< distances to all objects (calculated for every frame)
	std::vector<cv::Point3d> dist0ToShapes;					///< distance to all objects (postprocessor objects)
	std::vector<cv::Point3d> postProcessedDistances;		///< distance to all objects (postprocessed objects, ready to publish)
	std::vector<cv::Point3d> messageDistanceArray;			///< this array will be published
	nav_msgs::Path objectsInPath, objectsInPath_topic;		///< array of for better ROS compatibility
	imes_vision::object_finder topicMessage;				///< feature detector message



	/**
	 * @brief Outputs the properties (type/depth/channels) of a cv::Mat to console.
	 * @param image image
	 */
	void matInfoConsole(cv::Mat image);

	/**
	 * @brief Loads a image from storage.
	 * @param path Path to the image.
	 * @param resize_height Will resize the Mat to a defined height. width will automatically fit the original aspect ratio.
	 * @param floating Set to true if the Mat shall be 64F. Default (=false) is 8U.
	 */
	cv::Mat loader(std::string path, int resize_height, bool floating);

	/**
	 * @brief Trys to find a template in an image.
	 * @param img The input image.
	 * @param tpl The template to look for - The template-size should be smaller or equal than the size of the object in the image.
	 */
	cv::Mat templateMatcher(cv::Mat img, cv::Mat tpl);

	/**
	 * @brief Converts a one channel Mat to a three channel mat. Useful when colored things need to be shown on a grayscale image.
	 * @param grayMatsingle channel input image.
	 */
	cv::Mat OneCHtoThreeCH(cv::Mat grayMat);

	/**
	 * @brief Draws contours stored in a vector in a Mat.
	 * @param ContourVec The vector containing the contours.
	 * @param ContourVecPointer You can specify a specific contour of the vector. Set to negative to draw all contours of the vector.
	 * @param Channel Set the number of channels for the output mat. Default is 1, higher values will create a three channel output Mat.
	 * @param OutSize Size of output Mat. Must be given, because the function will not set it automatically
	 */
	cv::Mat vectorDrawer(std::vector<std::vector<cv::Point> > ContourVec,
			int ContourVecPointer, int Channel, cv::Size OutSize);

	/**
	 * @brief Draws a picture in a picture (PIP). The small pictures are 1/4 of it's original size and on the right side of the fullscreen picture.
	 * @param SourceMat The Mat to shrink.
	 * @param DestinationMat The fullscreen image to draw the small ones on (will be copied to ROIs on that image).
	 * @param PIPposition. The vertical slot of the small image. "1" will be on to right corner. "2" under "1", "3" under "2"[...]. 10px border to right and the one above will automaticalls be added.
	 */
	void PIPer(cv::Mat SourceMat, cv::Mat& DestinationMat, int PIPposition);

	/**
	 * @brief Draws text in top left corner of a Mat. Text size is 12, font is Arial. Antialiasing is on.
	 * @param DestinationMat The Mat to draw the text on.
	 * @param Text The text to draw.
	 */
	void onScreenInfo(cv::Mat &DestinationMat, std::string Text);

	/**
	 * @brief converts the detector results to nav_msgs::Path format
	 * @param input output
	 */
	void packIntoNavMsg(nav_msgs::Path &input);

};
