/************************************************************************
 * @brief      Shape-Detection-class for IMES ButtlerBot Project.
 *
 * 			   - This is a part of the IMES ButtlerBot, a demo exercise
 * 			     for the YouBot
 *
 * @file       shape_matcher.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-03-13
 *
 * Gottfried Wilhelm Leibniz UniversitÃ¤t
 * Institut fÃ¼r Mechatronische Systeme
 * AppelstraÃŸe 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <imes_vision/shape_matcher.h>

using namespace std;

shape_matcher::shape_matcher() {

	this->referenceSet= false;
	//this->modeCount= 0;
	this->boundingMode= 1;

	this->Trackbar1= 77;		// minLength in % of img.cols
	this->Trackbar2= 12; 		// gap in % of img.cols
	this->Trackbar3= 4; 		// gap in % of img.cols
	this->Trackbar4= 100;		// ContourFilter Lower
	this->Trackbar5= 2800;		// ContourFilter Higher
	this->Trackbar6= 501;		// Compliance
	this->Trackbar7= 57;		// Canny Lower
	this->Trackbar8= 155;		// Canny Higher
	this->Trackbar9= 15;		// Morph Kernelelement
	this->Trackbar10= 3;		// Dilate KernelElement
	this->Trackbar11= 21603;	// SizeFiler Lower
	this->Trackbar12= 100000;	// SizeFiler Higher
	this->Trackbar13= 100;		// Brightness

	aspectRatioFiltering = true;
	aspectDynRatioFiltering = false;
	sizeFiltering= false;
	contourFiltering= true;
	useDistanceCalc= true;

	roiSize = cv::Size(0,0);
	roiMiddle = cv::Point(0,0);
	roiOFFSET = cv::Point(0,0);
}

cv::Mat shape_matcher::OneCHtoThreeCH(cv::Mat grayMat) {

	cv::Mat grayMatVec[3] = {grayMat, grayMat, grayMat};
	cv::Mat BGRMat;

	cv::merge(grayMatVec, 3, BGRMat);
	return BGRMat;
}

cv::Point3d shape_matcher::getDistance(int VectorIndex) {

	return this->distanceVec[VectorIndex];
}

void shape_matcher::calculateDistance(cv::Mat Contour, cv::Mat image) {

	double _d[9] = {1,	0,	0, 0, -1, 0, 0,	0, -1}; //rotation: looking at -x axis
	cv::Mat rotM(3,3,CV_64FC1,_d);
	std::vector<cv::Point2f> imgP;
	imgP.clear();


// mode 2 -> RotatedRect
// mode 1 -> boundingRect

	cv::Point3d actualDist;
	cv::Rect box = cv::boundingRect(Contour);
	cv::RotatedRect boxR = cv::minAreaRect(Contour);

	// for roi-Correction! (otherwise x,y - calculation will fail!)
	if (roiSize.height > 0 && roiSize.width > 0) {
		roiOFFSET.x = roiMiddle.x  - 0.5 * roiSize.width;
		roiOFFSET.y = roiMiddle.y  - 0.5 * roiSize.height;
		box.x = box.x + roiOFFSET.x;
		box.y = box.y + roiOFFSET.y;
	}


	if(this->boundingMode == 2){
		this->shapeHeightVec.push_back(boxR.size.height);
		this->shapeWidthVec.push_back(boxR.size.width);

		if (boxR.size.width >= boxR.size.height){
			boxR.size.width  = boxR.size.height;
			boxR.size.height = boxR.size.width;
		}

		//P1 links-oben
		//P2 rechts-oben
		//P3 rechts-unten
		//P4 links-unten

		imgP.push_back(cv::Point(boxR.center.x - boxR.size.width/2,	boxR.center.y - boxR.size.height));
		imgP.push_back(cv::Point(boxR.center.x + boxR.size.width/2,	boxR.center.y - boxR.size.height));
		imgP.push_back(cv::Point(boxR.center.x + boxR.size.width/2,	boxR.center.y + boxR.size.height));
		imgP.push_back(cv::Point(boxR.center.x - boxR.size.width/2,	boxR.center.y + boxR.size.height));
	}
	else if(this->boundingMode == 1){
		this->shapeHeightVec.push_back(box.height);
		this->shapeWidthVec.push_back(box.width);



		// 		x		, 		y
		imgP.push_back(cv::Point(box.x,				box.y));
		imgP.push_back(cv::Point(box.x+box.width,	box.y));
		imgP.push_back(cv::Point(box.x+box.width,	box.y + box.height));
		imgP.push_back(cv::Point(box.x,				box.y + box.height));

	} else {
		ROS_ERROR("INVALIND BOUNDINGMODE! Press any key to continue.");
		cv::waitKey(0);
	}


	objP.clear();
    objP.push_back(cv::Point3d(0., 0., 0.));
    objP.push_back(cv::Point3d(this->objectWidth / 1000., 0.,  0.));
    objP.push_back(cv::Point3d(this->objectWidth / 1000., this->objectHeight / 1000.,  0.));
    objP.push_back(cv::Point3d(0., this->objectHeight / 1000.,  0.));

	//cv::Mat(objP).convertTo(objPM,CV_32F);
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);

    cv::solvePnP(cv::Mat(objP), cv::Mat(imgP), camera_matrix, distortion_coefficients, rvec, tvec, false);
    cv::Rodrigues(rvec,rotM);

    actualDist.x= tvec.at<double>(0,0) + double(this->objectWidth/2) / 1000.;
    actualDist.y= tvec.at<double>(1,0) + double(this->objectHeight/2) / 1000.;
    actualDist.z= tvec.at<double>(2,0);
    this->distanceVec.push_back(actualDist);
    ROS_INFO("Distance calculated: (X/Y/Z): %.2f/%.2f/%.2f", actualDist.z, actualDist.x,actualDist.y);	//ROI coords!
}

void shape_matcher::setCameraParams(int focalLength, cv::Mat cameraMatrix, cv::Mat distoritionCoeffs) {

	if (focalLength != 0)
		this->camFocalLength= focalLength;
	if (cameraMatrix.at<double>(0,0) > 0)
		cameraMatrix.copyTo(this->camera_matrix);
	if (distoritionCoeffs.at<double>(0,0) > 0)
		distoritionCoeffs.copyTo(this->distortion_coefficients);

	ROS_INFO("Camera parameters set!");
}

std::vector<std::vector<cv::Point> > shape_matcher::contourDetector(cv::Mat image1CH) {

	std::vector<std::vector<cv::Point> > ContourVec;
	cv::findContours(image1CH,	// input must be gray
					ContourVec, // Output-Vector (as created above)
					CV_RETR_EXTERNAL, // retrieve the external contours
					CV_CHAIN_APPROX_NONE); // all pixels of each contours

	// Eliminate too short or too long contours
	if (contourFiltering == true && (int)ContourVec.size() > 0) {

		std::vector<std::vector<cv::Point> >::iterator itc= ContourVec.begin();
		while (itc!=ContourVec.end()) {
			if ((int)(*itc).size() < Trackbar4 || (int)(*itc).size() > Trackbar5)
				itc= ContourVec.erase(itc);
			else
				++itc;
		}
	}

	if (this->sizeFiltering == true  && (int)ContourVec.size() > 0) {
		//sizeFiltering = Inhalt des mitgedrehten Rechtecks als Ausschlusskriterium
		std::vector<std::vector<cv::Point> >::iterator itc= ContourVec.begin();
		while (itc!=ContourVec.end()) {
			cv::RotatedRect box = cv::minAreaRect(*itc);
			if ((box.size.width * box.size.height < Trackbar11) || ((box.size.width * box.size.height) > Trackbar12))
				itc= ContourVec.erase(itc);
			else
				++itc;
		}
	}
	return ContourVec;
}

void shape_matcher::clearFilter(bool clear, cv::Size size) {

	cv::Mat tmp(size, CV_8UC1, cv::Scalar(255));

	if (clear == true) {
		this->filterMask= tmp;
		this->colorMask= tmp;
	}
}

cv::Mat shape_matcher::lineFilter(cv::Mat image) {

	cv::Mat out, tempBin;
	cv::Mat check;
	image.copyTo(tempBin);
	cv::Mat element5(5,5,CV_8U,cv::Scalar(1));

	check= OneCHtoThreeCH(image);

	image.copyTo(out);
	cv::dilate(tempBin, tempBin, element5);


	if (this->showWindows) {
		cv::namedWindow("Hough", CV_WINDOW_AUTOSIZE);
	}

	cv::createTrackbar("min. Line Length [%] : ", "Hough", &Trackbar1, 100, NULL);
	cv::createTrackbar("max gap [%]: ", "Hough", &Trackbar2, 25, NULL);
	cv::createTrackbar("accuracyInterval [0.1*%]: ", "Hough", &Trackbar3, 100, NULL);

	if (this->contourFiltering == true) {

		cv::createTrackbar("lower threshold ContourFilter: ", "Hough", &Trackbar4, 10000, NULL);
		cv::createTrackbar("higher threshold ContourFilter: ", "Hough", &Trackbar5, 10000, NULL);
	}

	cv::createTrackbar("compliance: ", "Hough", &Trackbar6, 1000, NULL);

	cv::createTrackbar("lower threshold Canny: ", "Hough", &Trackbar7, 1000, NULL);
	cv::createTrackbar("higher threshold Canny: ", "Hough", &Trackbar8, 1000, NULL);

	cv::createTrackbar("Element Morph: ", "Hough", &Trackbar9, 25, NULL);
	cv::createTrackbar("Element Dilataion: ", "Hough", &Trackbar10, 10, NULL);

	if (this->sizeFiltering == true) {

		cv::createTrackbar("lower threshold sizeFilter: ", "Hough", &Trackbar11, 100000, NULL);
		cv::createTrackbar("higher threshold sizeFilter: ", "Hough", &Trackbar12, 100000, NULL);
	}

	cv::createTrackbar("Brigthness: ", "Hough", &Trackbar13, 240, NULL);

	std::vector<cv::Vec4i> lines;
	std::vector<cv::Vec4i> lines2;
	double deltaRho=	1;
	double deltaTheta=	(M_PI/180);	// will only detect vertical (=0) and horizontal (=pi/2) lines!
	int minVote=		10;
	double minLength=	0.01*Trackbar1*image.cols;
	double maxGap=		0.01*Trackbar2*image.cols;


	cv::Mat imageROI(image.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat polyROI(image.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat polyMask(image.size(), CV_8UC1, cv::Scalar(0));
	std::vector <cv::Point> boxPoints, polyPoints;
	std::vector< std::vector <cv::Point> > polyPointsArray;
	boxPoints.clear();

	int accuracyInterval= (image.rows * 0.001 * Trackbar3 ); // would be 1% = 4px when image.height is 480px
	double m;

	cv::HoughLinesP(tempBin,lines,deltaRho, deltaTheta, minVote,minLength, maxGap);

	for (uint i=0; i < lines.size() ; i++) {

			//						([x1]	[y1])	,			([x2]		[y2])
			cv::line(check, cv::Point(1,lines[i][1]),cv::Point(image.cols,lines[i][3]), cv::Scalar(0,0,255), 1, 8 );
			//cv::line(this->filterMask, cv::Point(1,lines[i][1]),cv::Point(image.cols,lines[i][3]), cv::Scalar(0), 1, 8 );

			boxPoints.clear();
			boxPoints.push_back(cv::Point(0,lines[i][1]));
			boxPoints.push_back(cv::Point(image.cols,lines[i][3]));

			// Nu version with bounding Polygon (not Rectangle!)
			polyPoints.clear();
			polyPointsArray.clear();																// TODO:
			polyPoints.push_back(cv::Point(0,(lines[i][1] + accuracyInterval)));					// to correct -> 0
			polyPoints.push_back(cv::Point(image.cols,(lines[i][3] + accuracyInterval)));			// to correct -> image.cols
			polyPoints.push_back(cv::Point(image.cols,(lines[i][3] - accuracyInterval)));			// to correct -> image.cols
			polyPoints.push_back(cv::Point(0,(lines[i][1] - accuracyInterval)));					// to correct -> 0
			polyPointsArray.push_back(polyPoints);
			cv::fillPoly(polyMask, polyPointsArray, cv::Scalar(255), 8);
			cv::fillPoly(check, polyPointsArray, cv::Scalar(255,255,0), 8);


			out.copyTo(polyROI, polyMask);

			// detect smaller lines in imageROI and overpint them black in original Frame
			cv::HoughLinesP(polyROI,lines2, 1, (M_PI/180), 2, 5, 0);
			for (uint j=0; j < lines2.size() ; j++) {

				if ((lines2[j][0]) == (lines2[j][2])) { // line vertical
					m= -1;
				}
				else {
					m= 1. * abs(((lines2[j][3])-(lines2[j][1]))/((lines2[j][2])-(lines2[j][0])));
				}																							// to improve: use m only if this is wanted!!!!

				if ((m < 2) && (m > -1)) {
					//							([x1]		[y1])	,				([x2]		[y2])
					cv::line(check, cv::Point(lines2[j][0],lines2[j][1]),cv::Point(lines2[j][2],lines2[j][3]), cv::Scalar(255,0,0), 2, 8 );
					cv::line(out, cv::Point(lines2[j][0],lines2[j][1]),cv::Point(lines2[j][2],lines2[j][3]), cv::Scalar(0,0,0), 2, 8 );
					cv::line(this->filterMask, cv::Point(lines2[j][0],lines2[j][1]),cv::Point(lines2[j][2],lines2[j][3]), cv::Scalar(0), 3, 8 );
				}
			}
	}
	if (this->showWindows) {
		cv::imshow("Hough", check);
	}

return out;
}

cv::Mat shape_matcher::colorFilter(cv::Mat image, cv::Scalar hsv_min, cv::Scalar hsv_max, int maskMode) {

	std::vector <cv::Rect> boxVec, colorROI, cleanedROI;
	std::vector <cv::Point> bxPoints;

	cv::Mat boxMat(image.size(), CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat smoothedFrame;

	cv::GaussianBlur(image, smoothedFrame, cv::Size(5, 5), 2);

	// convert BGR to HSV
	cv::Mat hsv_frame(image.size(), CV_8UC3, cv::Scalar(0));
	cv::cvtColor(smoothedFrame, hsv_frame ,CV_BGR2HSV);

	// execute the filtering
	cv::Mat mask(image.size(), CV_8UC3, cv::Scalar(0)); // Used as mask in origFrame
	cv::inRange(hsv_frame, hsv_min, hsv_max, mask);

	switch (maskMode) {
		case 1: {	// Returns the mask
			cv::Mat FilterKernel(9,9,CV_8U,cv::Scalar(255));
			cv::erode(mask, mask, FilterKernel);
			this->colorMask= mask;
			return mask;
			break;
		}
		case 2: {	// Returns a wide bounding box around the colored areas
			cv::Mat FilterKernel(9,9,CV_8U,cv::Scalar(255));
			cv::erode(mask, mask, FilterKernel);
			image.copyTo(boxMat, mask);


			// Konturen der Farbbereiche
			std::vector<std::vector<cv::Point> > ContourVec;
			cv::findContours(mask, ContourVec, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			// find the contours
			for (std::vector <std::vector <cv::Point> >::iterator it = ContourVec.begin(); it < ContourVec.end(); it++ ){
				if (it->size() > 50) {
					// convert the contour point to a matrix
					std::vector <cv::Point> pts = *it;
					cv::Mat pointsMatrix = cv::Mat(pts);
				}
			}
			std::cout << "Number of colored Areas: " << ContourVec.size() << std::endl;

			boxVec.clear();
			colorROI.clear();
			cleanedROI.clear();
			bxPoints.clear();
			for (uint i =0; (i < ContourVec.size());i++) {
				cv::Rect box= cv::boundingRect(cv::Mat(ContourVec[i])); // Rect around shape
				boxVec.push_back(box);
				//cv::rectangle(boxMat,box,cv::Scalar(0,255,0),3);
			}

			int accuracyInterval= image.cols*0.02;
			for (uint j=0; j < boxVec.size(); j++) {
				for (uint k=0; k < boxVec.size(); k++) {
					if (((boxVec[j].width) >= (boxVec[k].width - accuracyInterval)) && ((boxVec[j].width) <= (boxVec[k].width + accuracyInterval)) && ((boxVec[j].x) >=(boxVec[k].x - accuracyInterval)) && ((boxVec[j].x) <= (boxVec[k].x + accuracyInterval))) {
						//Build new Rect around k and j-Rects:
						cv::Point k1= cv::Point((boxVec[k].x), (boxVec[k].y));
						cv::Point k3= cv::Point((boxVec[k].x + boxVec[k].width), (boxVec[k].y + boxVec[k].height));
						cv::Point j1= cv::Point(boxVec[j].x,boxVec[j].y);
						cv::Point j3= cv::Point((boxVec[j].x + boxVec[j].width), (boxVec[j].y + boxVec[j].height));
						bxPoints.push_back(k1);
						bxPoints.push_back(k3);
						bxPoints.push_back(j1);
						bxPoints.push_back(j3);
						cv::Rect bx= cv::boundingRect(cv::Mat(bxPoints));
						//cv::rectangle(boxMat,bx,cv::Scalar(0,0,255),2);
						colorROI.push_back(bx);
						bxPoints.clear();
					}
				}

			}
			std::cout << "Built " << colorROI.size() << " big color-areas " ;

			for (uint z=0; z < colorROI.size(); z++) {
				std::cout << "size: " << colorROI.size() << std::endl;
				for (uint y=0; y < colorROI.size(); y++) {
					if (colorROI[z].contains(cv::Point(colorROI[y].x + colorROI[y].width/2,colorROI[y].y - colorROI[y].height/2))) {
						std::cout << "erasing element: " << y << std::endl;
						colorROI.erase((colorROI.begin()+y));

					}
				}
			}
			std::cout << " filtered to " << colorROI.size() << std::endl;

/*
			// draw biggest bounding boxes
			for (uint l=0; l <colorROI.size(); l++) {
				for (uint m=0; m <colorROI.size(); m++) {
					if ((((colorROI[l]) & (colorROI[m])).area()) > 0) {
						cleanedROI.push_back((colorROI[l]) | (colorROI[m]));
					}
				}
			}
			//
			for (uint z=0; z < cleanedROI.size(); z++) {
				std::cout << "size: " << cleanedROI.size() << std::endl;
				for (uint y=0; y < cleanedROI.size(); y++) {
					if (cleanedROI[z].contains(cv::Point(cleanedROI[y].x,cleanedROI[y].y))) {
						std::cout << "erasing element: " << y << std::endl;
						cleanedROI.erase((cleanedROI.begin()+y));

					}
				}
			}

*/


			for (uint x=0; x < colorROI.size(); x++) {
				cv::rectangle(boxMat,colorROI[x],cv::Scalar(0,0,255),1);
			}
			this->colorMask=boxMat;
			return boxMat;
			break;
		}

		case 3: {	// Returns a wide bounding box around the colored areas
			cv::Mat FilterKernel(9,9,CV_8U,cv::Scalar(255));
			cv::erode(mask, mask, FilterKernel);
			// Konturen der Farbbereiche
			std::vector<std::vector<cv::Point> > ContourVec;
			ContourVec= contourDetector(mask);
			std::cout << "Number of colored Areas: " << ContourVec.size() << std::endl;
			cv::Mat boxMat(image.size(), CV_8U, cv::Scalar(0));
			for (uint i =0; (i < ContourVec.size());i++) {
				cv::Rect box= cv::boundingRect(cv::Mat(ContourVec[i])); // Rect around shape
				cv::rectangle(boxMat,box,cv::Scalar(255),2);
			}
			this->colorMask= boxMat;
			return boxMat;
			break;
		}


		default: {	// Return the colored parts of the source-image
			cv::Mat result(image.size(), CV_8UC3, cv::Scalar(255,255,255));
			image.copyTo(result, mask);
			this->colorMask= result;
			return result;
		}
	}
}

cv::Mat shape_matcher::BGR2BIN(cv::Mat imageMat, int mode, bool equalizing) {

	cv::Mat binMat;
	cv::Mat grayMat;

	// BGR2GRAY
	cv::cvtColor(imageMat, grayMat, CV_BGR2GRAY);

	// EQUALIZING
	if (equalizing == true)
		cv::equalizeHist(grayMat, grayMat);

	// GRAY2BIN: 3 Modes
		switch (mode) {
			case 2: {
				// Apply Canny algorithm
				cv::Mat contours;
				cv::Canny(grayMat, // gray-level image
							binMat, // output contours
							Trackbar7, // low threshold
							Trackbar8); // high threshold
				break;
			}
			case 1: {
				// GRAY2BIN (BIN has also 8 bit, but only 2 values)
				cv::adaptiveThreshold(grayMat, binMat, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 15, 10);
				break;
			}
			default: {
				// GRAY2BIN (BIN has also 8 bit, but only 2 values)
				cv::threshold(grayMat, binMat, 70, 255, CV_THRESH_BINARY_INV);
				break;
			}
		}
		this->binFrame= binMat;
	return binMat;
}

std::vector<std::vector<cv::Point> > shape_matcher::huller(std::vector<std::vector<cv::Point> > ContourVec) {

	std::vector<std::vector<cv::Point> >HullVec;
	std::vector<cv::Point> Hull;

	for (uint i=0; i < ContourVec.size(); i++) {
		cv::convexHull(cv::Mat(ContourVec[i]),Hull);
		HullVec.push_back(Hull);
	}
	return HullVec;
}

bool shape_matcher::setReference(std::string path, int height, int width) {

	this->referenceSet= false;

	// Get a reference shape:
	try {
		ROS_INFO_STREAM("Shape_path is: " << path);
		this->refShapeImg = cv::imread(path);
	}
	catch (cv::Exception &ex) {
		ROS_ERROR("--(!) Error reading Reference: %s ", ex.what());
		cv::waitKey();
	}

	if (!refShapeImg.data) {
		ROS_ERROR("--(!) Error reading Reference ");
	}
	cv::Mat Rev= BGR2BIN(this->refShapeImg, 5, 1);
	this->refContourVec.clear();
	this->refContourVec.push_back(contourDetector(Rev)[0]);

	this->objectWidth= width;
	this->objectHeight= height;

	// User-Output
	ROS_INFO("%d Reverence loaded and converted.", (int)refContourVec.size());
	this->referenceSet= true;

	return true;
}


void shape_matcher::setBoundingMode(int mode) {

	switch (mode) {
		case 1: this->boundingMode= 1;	// boundingRect
				break;
		case 2: this->boundingMode= 2;	// minRotatedRect
				break;

		default:this->boundingMode= 1;
				break;
	}

}


cv::Mat shape_matcher::binFilter(cv::Mat image) {

	cv::Mat filtered1(image.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat filtered2(image.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat element15(Trackbar9, Trackbar9, CV_8U, cv::Scalar(255));
	cv::Mat element3(Trackbar10, Trackbar10, CV_8U, cv::Scalar(255));

	cv::morphologyEx(image, filtered1, cv::MORPH_CLOSE, element15);
	cv::dilate(filtered1, filtered2, element3);

	filtered2.copyTo(this->filteredBinMat);

	return filtered2;
}


void shape_matcher::match(cv::Mat image) {

	// clear height, width and distance vectors:
	this->shapeHeightVec.clear();
	this->shapeWidthVec.clear();
	this->distanceVec.clear();

	// set Filtering if you wanna use it!!!!!!!!
	this->ContourVec= contourDetector(image);

	if (this->refContourVec.size() == 0) {
		ROS_ERROR("Please provide a reference-contour in the reverence-vector!");
	}

	if (this->refContourVec.size() > 1)
		ROS_INFO("This matcher will only use the first contour of the reverence-vector!");

	double compliance;
	std::vector<std::vector<cv::Point> >::iterator itc= ContourVec.begin();

	double shapeAspect;
	double shapeAspectHW;
	double shapeAspectWH;

	while ((itc != ContourVec.end())) {

		compliance = cv::matchShapes(cv::Mat(this->refContourVec[0]),cv::Mat(*itc), CV_CONTOURS_MATCH_I2, 0);
		//std::cout << "Compliance: " << compliance << std::endl;
		if (compliance > Trackbar6 * 0.01) {
			itc = ContourVec.erase(itc);
		} else {

			if (this->aspectRatioFiltering) {
				cv::Rect box = cv::boundingRect(cv::Mat(*itc)); // Rect around shape
				shapeAspect = (1. * (box.height) / (1. * (box.width)));
				if ((box.width >= box.height) || (shapeAspect > 5) || (shapeAspect < 3)) {
					itc = ContourVec.erase(itc);
				} else {
					// Time to calculate distance!
					// 1) Get height and width in pixels
					if (this->useDistanceCalc)
						calculateDistance(cv::Mat(*itc), image);
					++itc;
				}
			}
			if (this->aspectDynRatioFiltering) {
				cv::RotatedRect box = cv::minAreaRect(cv::Mat(*itc)); // Rect around shape
				shapeAspectHW = (1. * (objectHeight) / (1. * (objectWidth)));
				shapeAspectWH = (1. * (objectWidth) / (1. * (objectHeight)));
				double WH = box.size.width / box.size.height;
				double HW = box.size.height / box.size.width;
				if (((WH > shapeAspectWH * (1+RatioPro/100)) && (WH < shapeAspectWH * (1-RatioPro/100))) || ((HW > shapeAspectHW * 1.2) && (HW > shapeAspectHW * 0.8))) {
					itc = ContourVec.erase(itc);
				} else {
					// Time to calculate distance!
					// 1) Get height and width in pixels
					if (this->useDistanceCalc)
						calculateDistance(cv::Mat(*itc), image);
					++itc;
				}

			}
			if ((this->aspectDynRatioFiltering == false) && (this->aspectRatioFiltering == false)){
				// Time to calculate distance!
				// 1) Get height and width in pixels
				if (this->useDistanceCalc)
					calculateDistance(cv::Mat(*itc), image);
				++itc;
			}
		}
	}
	//std::cout << this->distanceVec.size() << " distances stored." << std::endl;
}
