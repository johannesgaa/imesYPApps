/************************************************************************
 * @brief      ObjectFinder-class for IMES ButtlerBot Project.
 *
 * 			   - This is a part of the IMES ButtlerBot, a demo exercise
 * 			     for the YouBot. This class controlles the two-step-pipeline
 * 			     for identifing objects with a 2D webcam.
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

#include <imes_vision/ObjectFinder.h>

/*
 TO DO: in den callback alle zwischenschritte einbauen. Ebenfalls die Bildausgabe
 if line Filtering -> origFrame->lineFiltering->filtered Frame (PreProcessor)
 if permanent ShapeDetection -> ShapeProcessor
 if permanent FeatureDetection -> Feature Detector

 Insgesammt also in einzelene mpdule Kapseln die alle einzeln aktiviert und deaktiviert werden können!!!!

 */

/* Constructor */
ObjectFinder::ObjectFinder(std::string cameraTopic, std::string cameraInfoTopic, std::string cameraLink, std::string mode, bool showCVwindows) :
		my_shape_matcher(), my_feature_matcher(), it(nh) {

	// Start Initialisation..
	MODE = mode;
	ROS_INFO("%s initialising..", MODE.c_str());

	// some default values for filtering an so on..
	this->camIsWorking = false;
	this->filterFrameCounter = 0;
	this->postFrameCounter = 0;
	this->colorFiltering = false;
	this->lineFiltering = true;
	ROS_INFO("Line-filtering enabled.");
	this->sequencyLength = 2;
	this->preProcessorRdy = false;
	this->postProcessorRdy = false;
	this->serviceCalled = false;
	this->detectionInProgress = true;
	this->stage1complete = false;
	this->loopRate = new ros::Rate(10);
	this->cmdWaiting = false;

	this->roiMiddle = cv::Point(0, 0);
	this->roiSize = cv::Size(0, 0);

	// defaults for shape detection:
	my_shape_matcher.Trackbar1 = 76;
	my_shape_matcher.Trackbar2 = 12;
	my_shape_matcher.Trackbar3 = 4;
	my_shape_matcher.Trackbar4 = 100;
	my_shape_matcher.Trackbar5 = 2800;
	my_shape_matcher.Trackbar6 = 22;
	my_shape_matcher.Trackbar7 = 67;
	my_shape_matcher.Trackbar8 = 196;
	my_shape_matcher.Trackbar9 = 15;
	my_shape_matcher.Trackbar10 = 3;

	// to indicate that the camera calibration has not finished:
	this->calibrationDone = false;

	// check if running on youbot:
	if (showCVwindows == true) {
		ROS_INFO("%s will show image-windows (gui-mode).", MODE.c_str());
		this->showWindows = true;
		my_shape_matcher.showWindows = true;
		my_feature_matcher.showWindows = true;
		cv::namedWindow(("Cam_Stream"), CV_WINDOW_AUTOSIZE);
		cv::startWindowThread();
	} else {
		ROS_INFO("%s will show not image-windows (CMD-mode).", MODE.c_str());
		this->showWindows = false;
		my_shape_matcher.showWindows = false;
		my_feature_matcher.showWindows = false;
	}

	// Registering callback and service or topic
	CAMERALINK = cameraLink;
	this->camerainfo_sub = nh.subscribe(cameraInfoTopic, 1, &ObjectFinder::cameraInfoCallback, this);

	// Here come the mode to play: ObjectFinder will run as Topic or Service for Shape or Feature Detection
	if (strcmp(MODE.c_str(), "Shape_Detector_Topic") == 0) {
		this->image_sub = it.subscribe(cameraTopic, 1, &ObjectFinder::imageCallback_shape_topic, this);
		this->object_finder_publisher = nh.advertise<imes_vision::object_finder>("/imes_vision/bottleDetector", 1);

		// shape detection without any buffers:
		this->sequencyLength = 1;
	}
	if (strcmp(MODE.c_str(), "Shape_Detector_Service") == 0) {
		this->image_sub = it.subscribe(cameraTopic, 1, &ObjectFinder::imageCallback_shape_service, this);
		this->my_service = nh.advertiseService("/imes_vision/Shape_Detector", &ObjectFinder::Shape_Detector_Service, this);
	}
	if (strcmp(MODE.c_str(), "Feature_Detector_Service") == 0) {
		this->image_sub = it.subscribe(cameraTopic, 1, &ObjectFinder::imageCallback_feature_service, this);
		this->my_service = nh.advertiseService("/imes_vision/Feature_Detector", &ObjectFinder::Feature_Detector_Service, this);
	}

	// just to be shure all arrays are empty:
	stickerArray.clear();
	posStickerArray.clear();
	ContourVec0.clear();
	postProcessedArray.clear();
	CV0Boxes.clear();

	// Initialisation finished:
	ROS_INFO("%s started!", MODE.c_str());
}

/**
 * @brief This callback function receives the camera_info topic for a single time and shuts down the subscriber itself
 */
void ObjectFinder::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {

	this->imageSize = Size(cam_info->width, cam_info->height);

	this->cameraMatrix = Mat::zeros(3, 3, CV_64FC1);
	this->distCoeffs = Mat::zeros(4, 1, CV_64FC1);

	this->cameraMatrix.at<double>(0, 0) = cam_info->K[0];
	this->cameraMatrix.at<double>(0, 2) = cam_info->K[2];
	this->cameraMatrix.at<double>(1, 1) = cam_info->K[4];
	this->cameraMatrix.at<double>(1, 2) = cam_info->K[5];
	this->cameraMatrix.at<double>(2, 2) = cam_info->K[8];

	for (int idx = 0; idx < 4; idx++) {
		this->distCoeffs.at<double>(idx, 0) = cam_info->D[idx];
	}
	double focalLength = 640; // Simulator!

	if (this->distCoeffs.at<double>(0, 0) != 0) {
		double fovx, fovy, aspectRatio;
		Point2d principalPt;
		cv::calibrationMatrixValues(this->cameraMatrix, Size(800, 600), 5.4, 4.4, fovx, fovy, focalLength, principalPt, aspectRatio);
		ROS_INFO("Undistorting video enabled! Calculated focus length is: %4.4f.", focalLength);
	}

	my_shape_matcher.setCameraParams(focalLength, this->cameraMatrix, this->distCoeffs); // simulator:
	my_feature_matcher.setRobustness(2);
	//cv::waitKey(3);

	camerainfo_sub.shutdown();
	this->calibrationDone = true;
}

void ObjectFinder::imageCallback_feature_service(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	cb_mutex.lock();
	if (this->detectionInProgress) {
		cmdWaiting = false;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

			this->distoredFrame = cv_ptr->image;
			this->camIsWorking = true;

			if (this->calibrationDone && (this->distCoeffs.at<double>(0, 0) != 0)) {
				cv::undistort(this->distoredFrame, this->origFrame, this->cameraMatrix, this->distCoeffs, this->cameraMatrix);
			} else {
				this->distoredFrame.copyTo(this->origFrame);
			}

			// IMAGE RECOGNITION SEQUENCE!
			if (this->serviceCalled) {

				// Now: Do FeatureDetection or save result of shape detection
				if ((this->imagePipeline > 1) && (this->my_feature_matcher.referenceSet)) {
					ROS_INFO("Detecting..");
					std::vector<std::vector<cv::Point> > tmp, inGripper;
					std::vector<cv::Point> pntVec;
					cv::Point tP;

					tP = cv::Point((int) (0.2 * this->origFrame.cols), (int) (-0.3 * this->origFrame.rows));
					pntVec.push_back(tP);
					tP = cv::Point((int) (0.8 * this->origFrame.cols), (int) (1.0 * this->origFrame.rows));
					pntVec.push_back(tP);

					inGripper.push_back(pntVec);
					FeatureProcessor(inGripper);
					//cv::waitKey(0);
				}

				// In "any" Case show us some nice Mats..
				if (this->showWindows) {
					cout << "..";
					Displayer(this->origFrame.size());
				}

				// done!
				this->detectionInProgress = false;
			}
		}

		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	} else {
		if (cmdWaiting == false && calibrationDone) {
			cmdWaiting = true;
			ROS_INFO("Waiting for clients..");
			cout << endl;
		}
		ros::Duration(1.0).sleep();
	}
	cb_mutex.unlock();
}

void ObjectFinder::imageCallback_shape_topic(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;
	cb_mutex.lock();

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		this->camIsWorking = true;

		// IMAGE RECOGNITION SEQUENCE!
		if (object_finder_publisher.getNumSubscribers() > 0) {
			this->cmdWaiting = false;
			this->distoredFrame = cv_ptr->image;

			if (this->calibrationDone && (this->distCoeffs.at<double>(0, 0) != 0)) {
				cv::undistort(this->distoredFrame, this->origFrame, this->cameraMatrix, this->distCoeffs, this->cameraMatrix);
			} else {
				this->distoredFrame.copyTo(this->origFrame);
			}

			this->actualProduct.positiveShapeResults = 0;
			this->actualProduct.position.clear();

			if (my_shape_matcher.referenceSet) {
				std::vector<std::vector<cv::Point> > tmp;

/*
				// PreProcessor gets ROI with all objects to detect and erase lines:
				int x = 0;
				int y = 0;
				int width = 0;
				int height = 0;
				if (nh.getParam("/imes_vision/bottleDetector/ROIx", x) && nh.getParam("/imes_vision/bottleDetector/ROIy", y) && nh.getParam("/imes_vision/bottleDetector/ROIwidth", width) && nh.getParam("/imes_vision/bottleDetector/ROIheight", height)) {
					cv::Mat roi = this->origFrame(cv::Rect(x,y, width, height));
					roi.copyTo(this->origFrame);
				}
*/
				PreProcessor(this->origFrame, 0);
				// Shape- and Postprocessor get only a really focused roi around the centered object:
				if (roiSize.height > 0 && roiSize.width > 0) {
					cv::Mat roi = this->filteredFrame(cv::Rect(roiMiddle.x - 0.5 * roiSize.width, roiMiddle.y - 0.5 * roiSize.height, roiSize.width, roiSize.height));
					roi.copyTo(this->filteredFrame);
				}
				PostProcessor(ShapeProcessor(this->filteredFrame), 0);
				tmp.clear();
				for (uint t = 0; t < this->postProcessedArray.size(); t++) {
					tmp.push_back(this->postProcessedArray[t]);
				}

				this->stage1complete = true;

				// In "any" Case show us some nice Mats..
				if (this->showWindows) {
					Displayer(this->origFrame.size());
				}

				this->actualProduct.positiveShapeResults = this->postProcessedArray.size();
				this->actualProduct.position.clear();

				for (uint z = 0; z < this->postProcessedArray.size(); z++) {
					this->actualProduct.position.push_back(cv::Point3d(this->postProcessedDistances[z].x, this->postProcessedDistances[z].y, this->postProcessedDistances[z].z));
				}
				publishMessage();
				this->stage1complete = false;
				this->postFrameCounter = 0;
				loopRate->sleep();
			}

			loopRate->sleep();
		} else {
			if (cmdWaiting == false && calibrationDone) {
				cmdWaiting = true;
				ROS_INFO("Waiting for subscribers..");
				cout << endl;
			}
			ros::Duration(0.5).sleep();
		}
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cb_mutex.unlock();
}

void ObjectFinder::imageCallback_shape_service(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;


	if (this->detectionInProgress) {
		this->cmdWaiting = false;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			this->distoredFrame = cv_ptr->image;
			this->camIsWorking = true;

			// IMAGE RECOGNITION SEQUENCE!
			if (this->serviceCalled) {
				cb_mutex.lock();
				ros::spinOnce();
				if (this->calibrationDone && (this->distCoeffs.at<double>(0, 0) != 0)) {
					cv::undistort(this->distoredFrame, this->origFrame, this->cameraMatrix, this->distCoeffs, this->cameraMatrix);
				} else {
					this->distoredFrame.copyTo(this->origFrame);
				}

				this->actualProduct.positiveShapeResults = 0;
				this->actualProduct.position.clear();
				if (my_shape_matcher.referenceSet) {

					std::vector<std::vector<cv::Point> > tmp;
					cout << "..";
					if (this->imagePipeline > 0) {
						PreProcessor(this->origFrame, this->sequencyLength);
						if (this->filterFrameCounter == this->sequencyLength) {
							this->filterFrameCounter = 0;
							cout << "..";

							PostProcessor(ShapeProcessor(this->filteredFrame), 2 * this->sequencyLength);
						}

						if (this->postFrameCounter >= 2 * this->sequencyLength) {
							tmp.clear();
							for (uint t = 0; t < this->postProcessedArray.size(); t++) {
								tmp.push_back(this->postProcessedArray[t]);
							}
							this->stage1complete = true;
						}
					}

					// In "any" Case show us some nice Mats..
					if (this->showWindows) {
						cout << "..";
						Displayer(this->origFrame.size());
					}

					if (this->stage1complete == true) {
						// save results!
						this->actualProduct.positiveShapeResults = this->postProcessedArray.size();
						this->actualProduct.position.clear();
						for (uint z = 0; z < this->postProcessedArray.size(); z++) {
							this->actualProduct.position.push_back(cv::Point3d(this->postProcessedDistances[z].x, this->postProcessedDistances[z].y, this->postProcessedDistances[z].z));
						}
						cout << endl;

						this->stage1complete = false;
						this->detectionInProgress = false;
						this->postFrameCounter = 0;
					}
				}
				cb_mutex.unlock();
			}

		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	} else {
		if (cmdWaiting == false && calibrationDone) {
			cmdWaiting = true;
			ROS_INFO("Waiting for clients..");
			cout << endl;
		}
		ros::Duration(1.0).sleep();
	}
}

void ObjectFinder::packIntoNavMsg(nav_msgs::Path &input) {

	geometry_msgs::PoseStamped actualPose;
	//ros::Time stamp = ros::Time::now();

	// fill Path header
	actualPose.header.frame_id = CAMERALINK;
	input.header.stamp = ros::Time::now();
	input.header.seq = 0;
	input.header.frame_id = CAMERALINK;
	input.poses.clear();

	// fill path.poses
	for (int i = 0; i < (int) messageDistanceArray.size(); i++) {
		actualPose.header.stamp = ros::Time::now();
		actualPose.header.seq = i;
		actualPose.header.frame_id = CAMERALINK;
		actualPose.pose.position.x = messageDistanceArray[i].x;
		actualPose.pose.position.y = messageDistanceArray[i].y;
		actualPose.pose.position.z = messageDistanceArray[i].z;
		actualPose.pose.orientation.x = 0.;
		actualPose.pose.orientation.y = 0.;
		actualPose.pose.orientation.z = 0.;
		actualPose.pose.orientation.w = 1.;
		input.poses.push_back(actualPose);
		ROS_INFO("Calculated object distance: X: %2.2f, Y: %2.2f", actualPose.pose.position.x, actualPose.pose.position.y);
	}
}

void ObjectFinder::publishMessage() {

	if ((int) messageDistanceArray.size() > 0) {
		packIntoNavMsg(objectsInPath_topic);
		topicMessage.objectsFound = (int) messageDistanceArray.size();
		topicMessage.objectPoses = objectsInPath_topic;
		ROS_INFO("Found %d object(s) at Y: %2.2f", topicMessage.objectsFound, topicMessage.objectPoses.poses[0].pose.position.y);

	} else {
		ROS_WARN("No object Found..");
		topicMessage.objectsFound = 0;
		topicMessage.objectNames.clear();
		topicMessage.objectNames.push_back("none");
		topicMessage.objectPoses.poses.clear();
		topicMessage.objectXposition.clear();
		topicMessage.objectYposition.clear();
		topicMessage.featureCompliance = 0;
	}
	object_finder_publisher.publish(topicMessage);
	return;
}

bool ObjectFinder::Shape_Detector_Service(imes_vision::ObjectFinderService::Request &req, imes_vision::ObjectFinderService::Response &res) {

	ros::Rate loop_rate(10);
	this->filterFrameCounter = 0;
	this->postFrameCounter = 0;
	this->stage1complete = false;
	this->messageArray.clear();

	ROS_INFO("Object Finder Service called!");

	if (this->camIsWorking == false) {
		ROS_ERROR("Please make shure a 2D camera is connected and publishing pictures!");
		return false;
	} else {
		ROS_INFO("Serach-engine started!");
		ros::spinOnce();
		std::cout << "Parameters: " << req.shapePath << std::endl;
		my_shape_matcher.contourFiltering = false;
		my_shape_matcher.sizeFiltering = false;
		my_shape_matcher.aspectRatioFiltering = true;
		my_shape_matcher.setReference(req.shapePath, req.height, req.width);
		my_shape_matcher.contourFiltering = true;
		my_shape_matcher.aspectRatioFiltering = true;
		ROS_INFO("Shape reference set.");
		ROS_INFO("Searching for objects ..");


		// Setup seems to be okay: Starting object detection (see imgCB!):
		this->detectionInProgress = true;
		this->serviceCalled = true;
		while (this->detectionInProgress) {
			loop_rate.sleep();
			ros::spinOnce();
		}
		ROS_INFO(".. detection done! (%d objects found)", (int)this->messageDistanceArray.size());
		cout << endl;

		// When Detector did the job: give result
		if ((int) messageDistanceArray.size() > 0) {

			// ---- Provide ROI for other nodes -------
			int smallestX = origFrame.cols;
			int biggestX = 0;
			int smallestY = origFrame.rows;
			int biggestY = 0;

			// create a roi around all objects and store it as ros param
			for (int i = 0; i < (int)CV0Boxes.size(); i++) {
				if (CV0Boxes[i].x < smallestX) {
					smallestX = CV0Boxes[i].x;
				}
				if (CV0Boxes[i].x > biggestX) {
					biggestX = CV0Boxes[i].x + CV0Boxes[i].width;
				}
				if (CV0Boxes[i].y < smallestY) {
					smallestY = CV0Boxes[i].y;
				}
				if (CV0Boxes[i].y > biggestY) {
					biggestY = CV0Boxes[i].y + CV0Boxes[i].height;
				}
			}
			smallestX= smallestX - 30;
			if (smallestX < 0) {
				smallestX = 0;
			}
			smallestY = smallestY - 30;
			if (smallestY < 0) {
				smallestY = 0;
			}
			biggestX = biggestX + 30;
			if (smallestX > origFrame.cols) {
				smallestX = origFrame.cols;
			}
			biggestY = biggestY + 30;
			if (biggestY > origFrame.rows) {
				smallestY = origFrame.rows;
			}
			int tmpX = biggestX-smallestX;
			int tmpY = biggestY-smallestY;
			if (smallestX && smallestY && biggestX && biggestY) {
				nh.param("ROIx", smallestX, 0);
				nh.param("ROIy", smallestY, 0);
				nh.param("ROIwidth", tmpX, 0);
				nh.param("ROIheight", tmpY, 0);
			}
			ROS_INFO("Set ROI parameters: X: %d, Y: %d, width: %d, height: %d", smallestX, smallestY, biggestX-smallestX, biggestY-smallestY);
			// ----------------------------------------


			packIntoNavMsg(objectsInPath);
			res.objectsFound = (int) messageDistanceArray.size();
			res.objectPoses = objectsInPath;
			this->serviceCalled = false;
			return true;
		} else {
			res.objectsFound = 0;
			packIntoNavMsg(objectsInPath);
			res.objectNames.push_back("none");
			this->serviceCalled = false;
			return true;
		}
	}
	return false;
}

bool ObjectFinder::Feature_Detector_Service(imes_vision::ObjectFinderService::Request &req, imes_vision::ObjectFinderService::Response &res) {

	ros::Rate loop_rate(10);

	ROS_INFO("Object Finder Service called!");

	if (this->camIsWorking == false) {
		ROS_ERROR("Please make shure a 2D camera is connected and publishing pictures!");
		return false;
	} else {
		ROS_INFO("Serach-engine started!");
		// std::cout << "Parameters: " << req.searchType << req.className << req.name << req.shapePath << req.height << req.width << req.featurePath << req.numberOfPictures << std::endl;

		my_feature_matcher.setReference(req.featurePath, req.numberOfPictures);
		ROS_INFO("Feature reference set.");
		ROS_INFO("Searching for objects ..");

		this->detectionInProgress = true;
		this->serviceCalled = true;
		while (this->detectionInProgress) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		ROS_INFO(".. detection done! (%d object found, compliance: %2.2f)", actualProduct.positiveShapeResults, my_feature_matcher.highestComp);
		cout << endl;
		if (actualProduct.positiveShapeResults > 0) {
			res.objectsFound = actualProduct.positiveShapeResults;
			res.objectPoses.poses.clear();
			res.featureCompliance = my_feature_matcher.highestComp;
			this->serviceCalled = false;
			return true;
		} else {
			res.objectsFound = 0;
			res.objectPoses.poses.clear();
			res.objectNames.push_back("none");
			res.featureCompliance = -1;
			this->serviceCalled = false;
			return true;
		}
	}
	return false;
}

void ObjectFinder::matInfoConsole(cv::Mat image) {
	std::cout << "Type: " << image.type() << ", Depth: " << image.depth() << ", Channel: " << image.channels() << std::endl;
	return;
}

cv::Mat ObjectFinder::loader(std::string path, int resize_height, bool floating) {

	cv::Mat imageMat;

	// Loading
	imageMat = cv::imread(path); // "/home/andi/ros_stacks/surf_test/reference.png"
	if ((!imageMat.data) || (!imageMat.data)) {
		std::cout << "Error while loading one picture" << std::endl;
	}

	// Resizing
	if (resize_height > 0) {
		cv::resize(imageMat, imageMat, cv::Size((resize_height * imageMat.cols / imageMat.rows), resize_height));
	}

	// Floting
	if (floating == true) {
		cv::Mat grayImageMat;
		cvtColor(imageMat, grayImageMat, CV_BGR2GRAY);
		cv::Mat Mat64;
		grayImageMat.convertTo(Mat64, CV_64FC1, 1. / 255.);
		return Mat64;
	}
	return imageMat;
}

// This is not used and not tested.
cv::Mat ObjectFinder::templateMatcher(cv::Mat img, cv::Mat tpl) {

	cv::Point minloc, maxloc;
	double minval, maxval;

	cv::Size size = cv::Size((img.cols - tpl.cols + 1), (img.rows - tpl.rows + 1));

	/* create new image for template matching computation */
	cv::Mat res(size, CV_32F, cv::Scalar(0));

	/* choose template matching method to be used */
	cv::matchTemplate(img, tpl, res, CV_TM_SQDIFF);
	/*cvMatchTemplate(img, tpl, res, CV_TM_SQDIFF_NORMED);
	 cvMatchTemplate(img, tpl, res, CV_TM_CCORR);
	 cvMatchTemplate(img, tpl, res, CV_TM_CCORR_NORMED);
	 cvMatchTemplate(img, tpl, res, CV_TM_CCOEFF);
	 cvMatchTemplate(img, tpl, res, CV_TM_CCOEFF_NORMED);*/

	cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);

	/* draw red rectangle */
	cv::rectangle(img, cv::Point(minloc.x, minloc.y), cv::Point(minloc.x + tpl.cols, minloc.y + tpl.rows), CV_RGB(255, 0, 0), 1, 0, 0);

	/* display images */
	if (this->showWindows) {
		cv::namedWindow("reference", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("template", CV_WINDOW_AUTOSIZE);
		cv::imshow("reference", img);
		cv::imshow("template", tpl);

		/* wait until user press a key to exit */
		//cv::waitKey(0);
		/* free memory */
		cv::destroyWindow("reference");
		cv::destroyWindow("template");
	}
	return img;
}

cv::Mat ObjectFinder::OneCHtoThreeCH(cv::Mat grayMat) {

	cv::Mat grayMatVec[3] = { grayMat, grayMat, grayMat };
	cv::Mat BGRMat;

	cv::merge(grayMatVec, 3, BGRMat);
	return BGRMat;
}

cv::Mat ObjectFinder::vectorDrawer(std::vector<std::vector<cv::Point> > ContourVec, int ContourVecPointer, int Channel, cv::Size OutSize) {

	cv::Mat ContourMat(OutSize, CV_8U, cv::Scalar(0));

	cv::drawContours(ContourMat, // Destination
	ContourVec, // Source
	ContourVecPointer, // if < 0: draw all contours
	cv::Scalar(255), // in black
	CV_FILLED, 8); // with a thickness of 2

	// Resizing if needed
	cv::Mat tmp;
	cv::resize(ContourMat, tmp, cv::Size((480 * ContourMat.cols / ContourMat.rows), 480));
	ContourMat = tmp;

	// if a 3 channel Mat shoud be returned
	if (Channel > 1) {
		cv::Mat ContourVec[3] = { ContourMat, ContourMat, ContourMat };
		cv::Mat Contour3Mat;
		cv::merge(ContourVec, 3, Contour3Mat);
		return Contour3Mat;
	}
	// this is a 1 channel Mat
	return ContourMat;
}

void ObjectFinder::PIPer(cv::Mat SourceMat, cv::Mat& DestinationMat, int PIPposition) {

	cv::Mat PIPmat;
	cv::Size PIPsize = cv::Size(SourceMat.cols / 4, SourceMat.rows / 4);
	cv::Mat PIProi = DestinationMat(cv::Rect((DestinationMat.cols - PIPsize.width - 10), (PIPposition * 10 + ((PIPposition - 1) * PIPsize.height)), PIPsize.width, PIPsize.height));

	cv::resize(SourceMat, PIPmat, PIPsize);
	PIPmat.copyTo(PIProi);
}

void ObjectFinder::onScreenInfo(cv::Mat &DestinationMat, std::string Text) {

	std::stringstream ss;
	ss.str("");
	ss.precision(2);
	ss << std::fixed << Text;
	cv::putText(DestinationMat, ss.str(), cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 0, 0), 1, CV_AA);
	return;
}

bool ObjectFinder::setROI(cv::Point roiMiddle, cv::Size roiSize) {

	this->roiMiddle = roiMiddle;
	this->roiSize = roiSize;

	my_shape_matcher.roiMiddle = roiMiddle;
	my_shape_matcher.roiSize = roiSize;

	ROS_INFO("ROI set.");

	// TODO:
	// if roiSize is bigger than image or roi goes out of image ....

	return true;
}

/* Image Pipeline:

 1) Frame -> PreProcessor* -> Filtered Frame
 2) Filtered Frame -> shape Processor -> vector <POSITIVE shapes>
 3) Positive shapes -> PostProcessor* -> ROIs around Stickers
 4) StickerROIS -> FeatureProcessor -> vector <POSITIVE Products>
 5) Positive roducts -> dist.calc -> laserScan comparison

 A) Displayer (origFRame, FIltered Frames, positive Shapes, Sticker ROIs, Distances: red(for all POS shapes), green(for POS stickers)


 */

void ObjectFinder::PreProcessor(cv::Mat input, int filterSequencyLength) {

	cv::Mat output(input.size(), CV_8UC1, cv::Scalar(255));
	cv::Mat tmp(input.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat tmp2(input.size(), CV_8UC1, cv::Scalar(0));

	if (this->lineFiltering == true) {

		if (this->filterFrameCounter == 0) {
			my_shape_matcher.clearFilter(true, input.size());
		}
		tmp = my_shape_matcher.BGR2BIN(input, 1, 0);
		my_shape_matcher.lineFilter(tmp);

		this->filterFrameCounter++;

		if (this->filterFrameCounter >= filterSequencyLength) {
			(my_shape_matcher.BGR2BIN(input, 1, 0)).copyTo(tmp2, (my_shape_matcher.filterMask));
			output = (my_shape_matcher.binFilter(tmp2));
			output.copyTo(this->filteredFrame);
			this->preProcessorRdy = true;
		}
	}

	if (this->colorFiltering == true) {

		if (this->filterFrameCounter == 0) {
			my_shape_matcher.clearFilter(true, input.size());
		}
		tmp = my_shape_matcher.BGR2BIN(input, 1, 0);
		my_shape_matcher.colorFilter(tmp, cv::Scalar(43, 58, 66), cv::Scalar(75, 255, 178), 1); //for Green
		my_shape_matcher.colorFilter(tmp, cv::Scalar(-20, 0, 0), cv::Scalar(45, 255, 94), 1); // for Brown
		this->filterFrameCounter++;

		if (this->filterFrameCounter >= filterSequencyLength) {

			tmp = my_shape_matcher.BGR2BIN(input, 1, 0);
			tmp.copyTo(tmp2, (my_shape_matcher.colorMask));
			//cv::bitwise_xor(my_shape_matcher.BGR2BIN(input, 2, 0), my_shape_matcher.filterMask, output);
			output = (my_shape_matcher.binFilter(tmp2));
			this->filteredFrame = output;
			this->preProcessorRdy = true;
		}
	}

	if ((this->colorFiltering == false) && (this->lineFiltering == false)) {
		tmp = my_shape_matcher.BGR2BIN(input, 1, 0);
		output = my_shape_matcher.binFilter(tmp);
		this->filteredFrame = tmp2; // will be black, cause there is no line-filtering
		this->preProcessorRdy = true;
	}
}

std::vector<std::vector<cv::Point> > ObjectFinder::ShapeProcessor(cv::Mat input) {
	std::vector<std::vector<cv::Point> > output;

	output.clear();
	my_shape_matcher.distanceVec.clear();
	this->distToShapes.clear();
	// 1) Match.
	my_shape_matcher.match(input);
	//ROS_INFO("Found %d contours.", (int)my_shape_matcher.ContourVec.size());
	output = my_shape_matcher.ContourVec;
	this->distToShapes = my_shape_matcher.distanceVec;
	return output;
}

void ObjectFinder::PostProcessor(std::vector<std::vector<cv::Point> > input, int sequencyLength) {

	// This will store the number and positions of each product we're looking for
	productDescription actualProduct;

	std::vector<cv::Rect> CVBoxes;
	std::vector<cv::Point> BoxMiddle;
	bool knownShape;
	std::stringstream onScreenText;

	if (this->showWindows)
		cv::namedWindow("CHECK", CV_WINDOW_AUTOSIZE);

	cv::Mat tmpCheck;
	this->origFrame.copyTo(tmpCheck);

// For Every Frame:
	// Clear Arrays
	CVBoxes.clear();
	BoxMiddle.clear();

	// 2) Draw rects and calc middle
	for (uint j = 0; j < input.size(); j++) {
		cv::Rect box = cv::boundingRect(cv::Mat(input[j]));
		CVBoxes.push_back(box);
		cv::Point middle = cv::Point((CVBoxes[j].x + CVBoxes[j].width / 2), (CVBoxes[j].y + CVBoxes[j].height / 2));
		BoxMiddle.push_back(middle);

		cv::rectangle(tmpCheck, box, cv::Scalar(255, 0, 0), 1, 8);
		onScreenText.str(std::string());
		onScreenText << "X: " << this->distToShapes[j].x << " Y: " << this->distToShapes[j].y; // Z is calculated and here but not shown on screen!
		cv::putText(tmpCheck, onScreenText.str(), cv::Point((CVBoxes[j].x), (CVBoxes[j].y + CVBoxes[j].height + 20)), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 0, 0), 1, CV_AA);

	}
	//std::cout << "Sizes: CVBOXES/BOXMIDDLE: " << CVBoxes.size() << " / " << BoxMiddle.size() << std::endl;

	// 3a) If first iteration: Save variables as basis
	if (this->postFrameCounter == 0) {
		this->ContourVec0.clear();
		this->CV0Boxes.clear();
		this->dist0ToShapes.clear();
		for (uint z = 0; z < input.size(); z++) {
			this->ContourVec0.push_back(input[z]); // !!!!!!!!!!!!!!!
			this->dist0ToShapes.push_back(this->distToShapes[z]);
		}
		for (uint y = 0; y < CVBoxes.size(); y++)
			this->CV0Boxes.push_back(CVBoxes[y]); // !!!!!!!!!!!!!!!
	}
	// 3b) for each result check if middle is inside CV0's rects
	else {
		for (uint k = 0; k < CVBoxes.size(); k++) {
			knownShape = false;

			for (uint l = 0; l < CV0Boxes.size(); l++) {
				// 3c) if so: do nothing, if not: add to first CV (new shape detected)
				if (CV0Boxes[l].contains(BoxMiddle[k])) {
					knownShape = true;
					//cv::circle(tmpCheck,BoxMiddle[k],2,cv::Scalar(255,0,0),2,8);

					// 3d) Check which box is smaller and keep that one!
					// Vergleich der breite
					if (CVBoxes[k].width < CV0Boxes[l].width) {
						CV0Boxes[l] = CVBoxes[k];
						this->ContourVec0[l] = input[k];
						this->dist0ToShapes[l] = this->distToShapes[k];
						cv::circle(tmpCheck, BoxMiddle[k], 2, cv::Scalar(255, 0, 255), 2, 8);
						onScreenText.str(std::string());
						onScreenText << "X: " << this->distToShapes[k].x << " Y: " << this->distToShapes[k].y; // Z is calculated and here but not shown on screen!
						cv::putText(tmpCheck, onScreenText.str(), cv::Point((CVBoxes[k].x), (CVBoxes[k].y + CVBoxes[k].height + 20)), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 0, 255), 1, CV_AA);
					}
				}
			}
			if (knownShape == false) {
				CV0Boxes.push_back(CVBoxes[k]);
				ContourVec0.push_back(input[k]);
				dist0ToShapes.push_back(distToShapes[k]);
				cv::circle(tmpCheck, BoxMiddle[k], 2, cv::Scalar(0, 0, 255), 2, 8);
				onScreenText.str(std::string());
				onScreenText << "X: " << this->distToShapes[k].x << " Y: " << this->distToShapes[k].y; // Z is calculated and here but not shown on screen!
				cv::putText(tmpCheck, onScreenText.str(), cv::Point((CVBoxes[k].x), (CVBoxes[k].y + CVBoxes[k].height + 20)), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 0, 255), 1, CV_AA);

				// distance auf tmpCheck schreiben
			}

		}
	}
	onScreenText.str(std::string());
	onScreenText << "Frame " << this->postFrameCounter << " of " << sequencyLength;
	cv::putText(tmpCheck, onScreenText.str(), cv::Point(5, 15), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 0, 0), 1, CV_AA);
	this->postFrameCounter++;
	if (this->showWindows)
		cv::imshow("CHECK", tmpCheck);

	float tmpX, tmpY, tmpZ;

	if (this->postFrameCounter >= sequencyLength || sequencyLength == 0) {
		// 4) When sequency is reached: store CV and clear all Arrays;
		this->postProcessedArray = ContourVec0;
		this->postProcessedDistances = this->dist0ToShapes;
		messageDistanceArray.clear();
		for (int i = 0; i < (int) dist0ToShapes.size(); i++) {
			// dist0ToShapes uses a messed up coordinate system:
			// 		x as distance left
			//		y as distance top
			//		z as distance front
			// we need to set: x=z, y=x, z=y, before sending out the message to be ros-standard-compatible!
			tmpX = dist0ToShapes[i].x;
			tmpY = dist0ToShapes[i].y;
			tmpZ = dist0ToShapes[i].z;
			dist0ToShapes[i].x = tmpZ;
			dist0ToShapes[i].y = -1 * tmpX;
			dist0ToShapes[i].z = tmpY;
			messageDistanceArray.push_back(dist0ToShapes[i]);
		}
		this->postProcessorRdy = true;
		this->detectionInProgress = false;
	}
}

void ObjectFinder::FeatureProcessor(std::vector<std::vector<cv::Point> > input) {

	cv::Mat maskSticker(this->origFrame.size(), CV_8UC1, cv::Scalar(255));
	cv::Mat imageROI(this->origFrame.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat maskShape(this->origFrame.size(), CV_8UC1, cv::Scalar(255));
	cv::Mat output(this->origFrame.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	std::vector<cv::Mat> stickerMat;
	std::stringstream onScreenText;
	std::vector<cv::Rect> stickerAry;
	stickerAry.clear();
	this->stickerArray.clear();
	this->posStickerArray.clear();
	this->actualProduct.positiveShapeResults = 0;
	this->actualProduct.position.clear();
	stickerMat.clear();

	if (this->showWindows)
		cv::namedWindow("ROI", CV_WINDOW_AUTOSIZE);

	for (uint i = 0; i < (input.size()); i++) {

		// Creation of a ROI for the feature detection
		// ROI= Box around bottom half of the positive shape
		cv::Rect box = cv::boundingRect(cv::Mat(input[i])); // Rect around shape
		cv::Rect sticker = box; // Rect around Sticker
		sticker.y = box.y + (box.height / 2);
		sticker.height = box.height / 2;
		stickerAry.push_back(sticker);
		cv::rectangle(maskSticker, sticker, cv::Scalar(255), CV_FILLED);
		imageROI = (this->origFrame)(sticker); // Mat for Feature Detection and Matching
		stickerMat.push_back(imageROI);
		if (this->showWindows) {
			cv::imshow("ROI", imageROI);
		}

		/*
		 // Feature Matching
		 my_feature_matcher.setConfidenceLevel(0.01*Trackbar1);
		 my_feature_matcher.setMinDistanceToEpipolar(Trackbar2);
		 my_feature_matcher.setRatio(0.01f * Trackbar3);
		 //cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector(10);
		 */

		// Match the two images
		std::vector<cv::DMatch> matches;
		matches.clear();
		std::vector<cv::KeyPoint> keypoints1, keypoints2;
		keypoints1.clear();
		keypoints2.clear();

		cv::Mat image2;
		bool positiveFeatureMatch = my_feature_matcher.match(imageROI, image2, matches, keypoints1, keypoints2);

		// if this is the searched product add it's position to position-vecor and increase canBeSeen by 1.
		if (positiveFeatureMatch == true) {
			this->posStickerArray.push_back(sticker);
			this->actualProduct.positiveShapeResults = this->actualProduct.positiveShapeResults + 1;
			//this->actualProduct.position.push_back(my_shape_matcher.getDistance(i));
		}

		//imageROI.setTo(Scalar(255,255,255));
	}
	this->stickerArray.clear();
	for (uint z = 0; z < stickerMat.size(); z++)
		this->stickerArray.push_back(stickerAry[z]);

	if ((this->showWindows) && (stickerMat.size() > 0)) {
		int numOfROIs = stickerMat.size();
		int h = 0;
		int w = 0;

		for (int k = 0; k < numOfROIs; k++) {
			if (stickerMat[k].rows > h)
				h = stickerMat[k].rows;
			w = w + stickerMat[k].cols;
		}
		cv::Size MatSize;
		MatSize.height = h;
		MatSize.width = w;
		cv::Mat allStickers(MatSize, CV_8UC3, cv::Scalar(255, 255, 255));
		int initX = 0;

		for (int j = 0; j < numOfROIs; j++) {
			if (j > 0) {
				initX = initX + stickerMat[j - 1].cols;
			}
			cv::Mat tmp;
			tmp.resize((stickerMat[j].cols, stickerMat[j].rows));
			tmp = allStickers(cv::Rect(initX, 0, stickerMat[j].cols, stickerMat[j].rows));
			stickerMat[j].copyTo(tmp);
		}
		cv::imshow("ROI", allStickers);
	}
	this->detectionInProgress = false;
}

void ObjectFinder::Displayer(cv::Size inputSize) {
	std::stringstream onScreenText;
	cv::Size outputSize, PIPsize;

	PIPsize.height = 0.25 * inputSize.height;
	PIPsize.width = 0.25 * inputSize.width;
	outputSize.width = 1.25 * inputSize.width;
	outputSize.height = inputSize.height;

	cv::Mat output(outputSize, CV_8UC3, cv::Scalar(0, 255, 0));

	/* OUTPUT:
	 *  ______________________________________	   ___  ___
	 * |                            |         |	   / \  / \
	 * |	       				    |	 2    |		|   0.25
	 * |                     		|_________|		|  _\_/_
	 * |                     		|         |		|
	 * |					 		|	 3    |		|
	 * |             1        		|_________|	  camFrame.height
	 * |                     		|         |		|
	 * |					 		|	 4    |		|
	 * |                     		|_________|		|
	 * |                     		|         |		|
	 * |					 		|	 5    |	    |
	 * |____________________________|_________|	  _\_/_
	 *
	 * |<----camFrame.width-------->|<--0.25->|
	 */

	// ROI: (X,Y, WIDTH, HEIGHT):     X,                 Y,               width,            height
	cv::Mat roi1 = output(cv::Rect(0, 0, inputSize.width, inputSize.height));
	cv::Mat roi2 = output(cv::Rect(inputSize.width, 0, PIPsize.width, PIPsize.height));
	cv::Mat roi3 = output(cv::Rect(inputSize.width, PIPsize.height, PIPsize.width, PIPsize.height));
	cv::Mat roi4 = output(cv::Rect(inputSize.width, 2 * PIPsize.height, PIPsize.width, PIPsize.height));
	cv::Mat roi5 = output(cv::Rect(inputSize.width, 3 * PIPsize.height, PIPsize.width, PIPsize.height));

	// ROI1:
	this->origFrame.copyTo(roi1);
	if (roiSize.height && roiSize.width) {
		cv::rectangle(roi1, cv::Rect(roiMiddle.x-(roiSize.width/2), roiMiddle.y-roiSize.height/2, roiSize.width, roiSize.height), cv::Scalar(0,255,0), 2, 8, 0);
		cv::Mat roi1ex = roi1(cv::Rect(roiMiddle.x-(roiSize.width/2), roiMiddle.y-roiSize.height/2, roiSize.width, roiSize.height));
		cv::drawContours(roi1ex, this->postProcessedArray, -1, cv::Scalar(255, 0, 0), 2, 8); // positiveContours
	} else {
		cv::drawContours(roi1, this->postProcessedArray, -1, cv::Scalar(255, 0, 0), 2, 8); // positiveContours
	}

	// ROI2:
	cv::resize(OneCHtoThreeCH(my_shape_matcher.binFrame), roi2, PIPsize);

	// ROI3:
	if (this->preProcessorRdy == true)
		cv::resize(OneCHtoThreeCH(this->filteredFrame), roi3, PIPsize);

	// ROI4:
	if (this->postProcessorRdy == true) {
		cv::Mat tmp(inputSize, CV_8UC3, cv::Scalar(255, 255, 255));

		tmp = vectorDrawer(this->postProcessedArray, -1, 3, inputSize);
		cv::resize(tmp, roi4, PIPsize);
	}

	// ROI5:
	//cv::resize(Frame, roi5, PIPsize);

	// ROI1 extras:
	for (uint i = 0; i < this->stickerArray.size(); i++) {
		cv::rectangle(roi1, stickerArray[i], cv::Scalar(0, 0, 255), 1, 8, 0); // red Rects around Stickers
		onScreenText.str(std::string()); // Distances to objects
		onScreenText << "X: " << this->postProcessedDistances[i].x << " Y: " << this->postProcessedDistances[i].y;
		cv::putText(roi1, onScreenText.str(), cv::Point((stickerArray[i].x), (stickerArray[i].y + stickerArray[i].height + 20)), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 0, 255), 1, CV_AA);
	}

	for (uint j = 0; j < this->posStickerArray.size(); j++)
		cv::rectangle(roi1, posStickerArray[j], cv::Scalar(0, 255, 0), 1, 8, 0); // green Rects around POS.Stickers

	// Rects around ROIs:
	cv::rectangle(output, cv::Point(0, 0), cv::Point(inputSize.width, inputSize.height), cv::Scalar(128, 128, 128), 1, 8, 0); //roi1
	cv::rectangle(output, cv::Point(inputSize.width, 0), cv::Point(outputSize.width, PIPsize.height), cv::Scalar(128, 0, 0), 1, 8, 0); //roi2
	cv::rectangle(output, cv::Point(inputSize.width, PIPsize.height), cv::Point(outputSize.width, 2 * PIPsize.height), cv::Scalar(0, 128, 0), 1, 8, 0); //roi3
	cv::rectangle(output, cv::Point(inputSize.width, 2 * PIPsize.height), cv::Point(outputSize.width, 3 * PIPsize.height), cv::Scalar(0, 0, 128), 1, 8, 0); //roi4
	cv::rectangle(output, cv::Point(inputSize.width, 3 * PIPsize.height), cv::Point(outputSize.width, inputSize.height), cv::Scalar(128, 128, 128), 1, 8, 0); //roi5

	onScreenText.str(std::string());
	onScreenText << "PreFrame: " << this->filterFrameCounter << " / PostFrame: " << this->postFrameCounter;
	cv::putText(roi1, onScreenText.str(), cv::Point(5, 15), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 0, 0), 1, CV_AA);

	if (this->showWindows)
		cv::imshow("Cam_Stream", output);
}

