/************************************************************************
 * @brief      FeatureDetection-class for IMES ButtlerBot Project.
 *
 * 			   - This is a part of the IMES ButtlerBot, a demo exercise
 * 			     for the YouBot
 *
 * @file       feature_matcher.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2012-03-13
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <imes_vision/feature_matcher.h>


//CLASS CONSTRUCTOR:
feature_matcher::feature_matcher(): ratio(0.65), refineF(true), confidence(0.98), distance(3.0) {

	this->detector= new cv::SiftFeatureDetector(0.03, 10.);
	this->extractor= new cv::SiftDescriptorExtractor();

/*
	// SURF is the default feature
	  this->detector= new cv::SurfFeatureDetector(10., 3, 4, false);
	  this->extractor= new cv::SurfDescriptorExtractor();
*/

	  this->referenceSet=false;
	//setReference(name, path, cv::Mat());

		Trackbar1= 98;		// ConfidenceLevel
		Trackbar2= 1;	 	// MinDistanceToEpipolar
		Trackbar3= 65; 		// Ratio
		Trackbar4= 15;		// min Matches

		minimumMatches = 15;

	setRobustness(2);

};

void feature_matcher::setRobustness(int robustness, int neededPositiveMatches) {

	minimumMatches = neededPositiveMatches;

	switch (robustness) {
	case 4: {
		this->rsacTest = true;
		this->symTest = true;
		this->rTest21 = true;
		this->rTest12 = true;
		break;
	}
	case 3: {
		this->rsacTest = false;
		this->symTest = true;
		this->rTest21 = true;
		this->rTest12 = true;
		break;
	}
	case 2: {
		this->rsacTest = false;
		this->symTest = false;
		this->rTest21 = true;
		this->rTest12 = true;
		break;
	}
	default:
	case 1: {
		this->rsacTest = false;
		this->symTest = false;
		this->rTest21 = false;
		this->rTest12 = true;
		break;
	}
	}
	ROS_INFO_STREAM("Robustness set to " << robustness << ".");
}

void feature_matcher::setRobustness(int robustness) {

	switch (robustness) {
	case 4: {
		this->rsacTest = true;
		this->symTest = true;
		this->rTest21 = true;
		this->rTest12 = true;
		break;
	}
	case 3: {
		this->rsacTest = false;
		this->symTest = true;
		this->rTest21 = true;
		this->rTest12 = true;
		break;
	}
	case 2: {
		this->rsacTest = false;
		this->symTest = false;
		this->rTest21 = true;
		this->rTest12 = true;
		break;
	}
	default:
	case 1: {
		this->rsacTest = false;
		this->symTest = false;
		this->rTest21 = false;
		this->rTest12 = true;
		break;
	}
	}
	ROS_INFO_STREAM("Robustness set to " << robustness << ".");
}


void feature_matcher::setReference(std::string path, int numPics) {

	this->referenceSet = false;
	cv::Mat image;
	std::string filePath;
	std::vector<cv::KeyPoint> actualKeyPoints;
	cv::Mat actualDescriptors;
	std::stringstream tmpstr;

	int refMatWidth= 0;
	int refMatHeight= 0;

	this->numOfRefPics= numPics;

	// Delete old references
	this->referencePics.clear();
	this->refKeypoints.clear();
	this->refDescriptors.clear();
	info_lastMatch.referenceKeys = 0;

	if (numPics < 8) {
		std::cout << "You provided only " << numPics << " Pictures." << std::endl;
		std::cout << "By providing less than 8 to 12 pictures the robot may not be able to identify your object correctly! Press any key to continue anyway." << std::endl;
		cv::waitKey(0);
	}

	for (int i=1; i<= numPics; i++) {
		tmpstr.str("");
		tmpstr << path << "/" << i << ".bmp";
		filePath= tmpstr.str();
		image= cv::imread(filePath);
		std::cout << "Loading " << i << ". reference Picture ..." << std::endl;
		if ((!image.data) || (!image.data)) {
				std::cout << "Error while loading picture" << std::endl;
				cv::waitKey(0);
		}
		else {
			// Add image to reference image array
			(this->referencePics).push_back(image);

			if (image.rows > refMatHeight)
				refMatHeight=image.rows;
			refMatWidth= refMatWidth + image.cols;

			// Find Features and extract Descriptors, store both in their arrays
			detector->detect(referencePics.back(), actualKeyPoints);
			(this->refKeypoints).push_back(actualKeyPoints);
			info_lastMatch.referenceKeys += refKeypoints.back().size();
			extractor->compute(image, (this->refKeypoints).back(), actualDescriptors);
			(this->refDescriptors).push_back(actualDescriptors);
		}
		actualKeyPoints.clear();
	}


	int initX= 0;
	cv::Mat allRefs(cv::Size(refMatWidth,refMatHeight), CV_8UC3, cv::Scalar(128));
	for (uint z=0; z < this->referencePics.size(); z++) {

		if (z > 0){
			initX= initX + referencePics[z-1].cols;
		}
		cv::Mat tmp(cv::Size(referencePics[z].cols,referencePics[z].rows), CV_8UC3, cv::Scalar(128));
		//tmp.resize(referencePics[z].cols, referencePics[z].rows);
		tmp = allRefs(cv::Rect(initX, 0, referencePics[z].cols, referencePics[z].rows));
		referencePics[z].copyTo(tmp);
	}
	allRefs.copyTo(this->allReferences);

	if (this->showWindows) {
		//cv::namedWindow("FEATURE REF", CV_WINDOW_AUTOSIZE);
		//cv::imshow("FEATURE REF", allRefs);
	}

	std::cout << "Loading of picture set containing " << (this->referencePics.size()) << " pictures complete. " << std::endl;
	std::cout << "Total FeatureDescriptors: " << this->refDescriptors.size() << std::endl;
	this->referenceSet = true;
}



// if you want the F matrix to be recalculated
void feature_matcher::refineFundamental(bool flag) {
	refineF= flag;
};

// Clear matches for which NN ratio is > than threshold
// return the number of removed points
// (corresponding entries being cleared, i.e. size will be 0)
int feature_matcher::ratioTest(std::vector<std::vector<cv::DMatch> >& matches) {
	int removed=0;

	// for all matches
	for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator= matches.begin();
	                         matchIterator!= matches.end(); ++matchIterator) {

			// if 2 NN has been identified
			if (matchIterator->size() > 1) {

				// check distance ratio
				if ((*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio) {
					matchIterator->clear(); // remove match
	            	removed++;
				}
			}
			else { // does not have 2 neighbours
				matchIterator->clear(); // remove match
				removed++;
			}
	}
	return removed;
}

// Insert symmetrical matches in symMatches vector
void feature_matcher::symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
	                                const std::vector<std::vector<cv::DMatch> >& matches2,
	                                            std::vector<cv::DMatch>& symMatches) {

	// for all matches image 1 -> image 2
	for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin();
	                         matchIterator1!= matches1.end(); ++matchIterator1) {

			if (matchIterator1->size() < 2) // ignore deleted matches
				continue;

			// for all matches image 2 -> image 1
			for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator2= matches2.begin();
	                                matchIterator2!= matches2.end(); ++matchIterator2) {

				if (matchIterator2->size() < 2) // ignore deleted matches
					continue;


				// Match symmetry test
				if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  &&
	                                        (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {

					// add symmetrical match
					symMatches.push_back(cv::DMatch((*matchIterator1)[0].queryIdx,(*matchIterator1)[0].trainIdx,(*matchIterator1)[0].distance));
					break; // next match in image 1 -> image 2
				}
			}
	}
}


// Identify good matches using RANSAC
// Return fundemental matrix
cv::Mat feature_matcher::ransacTest(const std::vector<cv::DMatch>& matches,
	                                 const std::vector<cv::KeyPoint>& keypoints1,
	                                  const std::vector<cv::KeyPoint>& keypoints2,
	                                   std::vector<cv::DMatch>& outMatches) {

	// Convert keypoints into Point2f
	std::vector<cv::Point2f> points1, points2;
	for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it) {

	    // Get the position of left keypoints
		float x= keypoints1[it->queryIdx].pt.x;
		float y= keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x,y));
		// Get the position of right keypoints
		x= keypoints2[it->trainIdx].pt.x;
		y= keypoints2[it->trainIdx].pt.y;
		points2.push_back(cv::Point2f(x,y));
	}

	// Compute F matrix using RANSAC
	std::vector<uchar> inliers(points1.size(),0);
	cv::Mat fundemental;

	fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), // matching points
	                    					inliers,      // match status (inlier ou outlier)
	                    						CV_FM_RANSAC, // RANSAC method
	                    						distance,     // distance to epipolar line
	                    						confidence);  // confidence probability

	// extract the surviving (inliers) matches
	std::vector<uchar>::const_iterator itIn= inliers.begin();
	std::vector<cv::DMatch>::const_iterator itM= matches.begin();

	// for all matches
	for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
		if (*itIn) { // it is a valid match
			outMatches.push_back(*itM);
		}
	}

	std::cout << "Number of matched points (after cleaning): " << outMatches.size() << std::endl;

	if (refineF) { // The F matrix will be recomputed with all accepted matches
		// Convert keypoints into Point2f for final F computation
		points1.clear();
		points2.clear();

		for (std::vector<cv::DMatch>::const_iterator it= outMatches.begin();it!= outMatches.end(); ++it) {

			// Get the position of left keypoints
			float x= keypoints1[it->queryIdx].pt.x;
			float y= keypoints1[it->queryIdx].pt.y;
			points1.push_back(cv::Point2f(x,y));
			// Get the position of right keypoints
			x= keypoints2[it->trainIdx].pt.x;
			y= keypoints2[it->trainIdx].pt.y;
			points2.push_back(cv::Point2f(x,y));
		}

		// Compute 8-point F from all accepted matches
		fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), // matching points
	                                							CV_FM_8POINT); // 8-point method
		}

		return fundemental;
}

// Set the feature detector
void feature_matcher::setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect) {
	detector= detect;
}

// Set descriptor extractor
void feature_matcher::setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc) {
	extractor= desc;
}

// Set the minimum distance to epipolar in RANSAC
void feature_matcher::setMinDistanceToEpipolar(double d) {
	distance= d;
}

// Set confidence level in RANSAC
void feature_matcher::setConfidenceLevel(double c) {
	confidence= c;
}

// Set the NN ratio
void feature_matcher::setRatio(float r) {
	this->ratio= r;
}

// Match feature points using symmetry test and RANSAC
// returns fundemental matrix
bool feature_matcher::match(cv::Mat& image1, cv::Mat& image2, // input images
	                  std::vector<cv::DMatch>& matches, // output matches and keypoints
	                  std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2) {

	ros::Time startTime = ros::Time::now();
	cv::Mat matchesMat;
	std::stringstream onScreenText;
	// cleaning..
	int imageMatches;
	imageMatches= imageMatches+0;
	matches.clear();
	keypoints1.clear();
	highestComp = 0.;
/*
	cv::createTrackbar("ConfidenceLevel [%] : ", "FEATURE REF", &Trackbar1, 100, NULL);
	cv::createTrackbar("MinDistanceToEpipolar: ", "FEATURE REF", &Trackbar2, 10, NULL);
	cv::createTrackbar("Ratio [%]: ", "FEATURE REF", &Trackbar3, 100, NULL);
	cv::createTrackbar("Min. Matches: ", "FEATURE REF", &Trackbar4, 200, NULL);
*/
	// 1a. Detection of the SURF/SIFT features
	detector->detect(image1,keypoints1);

	std::cout << keypoints1.size() << " Keypoints extracted from Label" << std::endl;
	info_lastMatch.imageKeys = keypoints1.size();

	// 1b. Extraction of the SURF/SIFT descriptors
	cv::Mat descriptors1, descriptors2;
	extractor->compute(image1,keypoints1,descriptors1);
	//extractor->compute(image2,keypoints2,descriptors2);

	//std::cout << "descriptor matrix size: " << descriptors1.rows << " by " << descriptors1.cols << std::endl;

	cv::Mat tmp;
	this->allReferences.copyTo(tmp);

	onScreenText.str(std::string());
	onScreenText << "Keypoints on Scene-Label: " << keypoints1.size();
	cv::putText(tmp, onScreenText.str(), cv::Point(10, this->allReferences.rows - 50), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(200, 200, 0), 1, CV_AA);


	std::cout << "Matching with references (ratio test) : " << std::endl;
	printf("Ref1\tRef2\tRef3\tRef4\tRef5\tRef6\tRef7\tRef8");
	std::cout << std::endl;

	int featureMatches[2][numOfRefPics];
	int positiveRef;
	float highestMatch;
	std::vector<std::vector<cv::DMatch> > bestMatches;
	cv::Mat bestDescriptors;
	std::vector<cv::KeyPoint> bestKeypoints;
	std::stringstream resultText;
	positiveRef=-1;
	highestMatch=-1;
	resultText.str(std::string());

	for (uint i = 0; (i < (this->referencePics).size()); i++) {

		// take the right Pic/KP/Desc of the Array and store it in XXX2 <- only for easier handling
		keypoints2 = this->refKeypoints[i];
		descriptors2 = this->refDescriptors[i];
		image2= this->referencePics[i];

		// 2. Match the two image descriptors

		// Construction of the matcher
		cv::BruteForceMatcher<cv::L2<float> > matcher;

		// from image 1 to image 2
		// based on k nearest neighbours (with k=2)
		std::vector<std::vector<cv::DMatch> > matches1;
		matches1.clear();
		matcher.knnMatch(descriptors1, descriptors2, matches1, // vector of matches (up to 2 per entry)
				2); // return 2 nearest neighbours

		// from image 2 to image 1
		// based on k nearest neighbours (with k=2)
		std::vector<std::vector<cv::DMatch> > matches2;
		matches2.clear();
		matcher.knnMatch(descriptors2, descriptors1, matches2, // vector of matches (up to 2 per entry)
				2); // return 2 nearest neighbours

	                //std::cout << "Number of matched points 1->2: " << matches1.size() << std::endl;
	                //std::cout << "Number of matched points 2->1: " << matches2.size() << std::endl;
	                //cv::waitKey(0);

		int stableMatches= 0;
		stableMatches = stableMatches+0;

		// 3. Remove matches for which NN ratio is > than threshold
		if (rTest12) {
			// clean image 1 -> image 2 matches
			int removed = ratioTest(matches1);
			printf("%i", (int)(matches1.size() - removed));
			resultText << (int)(matches1.size() - removed);
			onScreenText.str(std::string());
			onScreenText << "Matches (1->2): " << (matches1.size() - removed);
			cv::putText(tmp, onScreenText.str(), cv::Point(((this->allReferences.cols / this->referencePics.size()) * i + 10),15), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(200, 200, 0), 1, CV_AA);
			stableMatches = matches1.size() - removed;
			featureMatches[0][i]= matches1.size() - removed;


			if (rTest21) {
				// clean image 2 -> image 1 matches
				removed = ratioTest(matches2);
				printf("/%i\t", (int)(matches2.size() - removed));
				resultText << "/" << (int)(matches2.size() - removed) << "  ";
				onScreenText.str(std::string());
				onScreenText << "Matches (1<-2): " << (matches2.size() - removed);
				cv::putText(tmp, onScreenText.str(), cv::Point(((this->allReferences.cols / this->referencePics.size()) * i + 10),35), CV_FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(200, 200, 0), 1, CV_AA);
				stableMatches = matches2.size() - removed;
				featureMatches[1][i]= matches2.size() - removed;

				// 4. Remove non-symmetrical matches
				if (symTest) {
					std::vector<cv::DMatch> symMatches;
					symmetryTest(matches1, matches2, symMatches);
					std::cout << "Number of matched points (symmetry test): "
							<< symMatches.size() << std::endl;
					stableMatches = symMatches.size();

					// 5. Validate matches using RANSAC
					if (rsacTest) {
						cv::Mat fundemental = ransacTest(symMatches, keypoints1,
								keypoints2, matches);
						stableMatches = matches.size();
					}
				}
			}
		}
		// Wenn die Anzahl an Matches beidseitig größer als 15 ist
		if ((featureMatches[0][i] > minimumMatches) && (featureMatches[1][i] > minimumMatches)) {

			if (((featureMatches[1][i]) + (featureMatches[0][i])) > highestMatch) {
				highestMatch= ((featureMatches[1][i]) + (featureMatches[0][i]));
				positiveRef= i;
				bestKeypoints=keypoints2;
				bestMatches= matches2;
				info_lastMatch.positiveMatches1 = featureMatches[0][i];
				info_lastMatch.positiveMatches2 = featureMatches[1][i];
				//cv::waitKey(0);
			}
		}
	}
	std::cout << std::endl;

	// Now look for highest positive Match!
	// TODO: die refPic links und rechts von RefPic[highestMatch] auf Matches überprüfen -> sollten auch Matches haben!!
	highestComp= highestMatch;

	if (positiveRef > -1) {
		std::cout << "The Object was found. Reference picture number " << positiveRef+1 << " gave the best results." << std::endl;

		if (this->showWindows) {
			cv::drawMatches(image1, keypoints1, // 1st image and its keypoints
					this->referencePics[positiveRef], bestKeypoints, // 2nd image and its keypoints
					bestMatches, // the matches
					matchesMat, // the image produced
					cv::Scalar(0, 255, 0)); // color of the lines
			onScreenText.str(std::string());
			onScreenText << "Ref1   Ref2   Ref3   Ref4   Ref5   Ref6   Ref7   Ref8";
			cv::putText(matchesMat, onScreenText.str(), cv::Point(image1.cols/2 - 200, image1.rows/2 +30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(10, 10, 255), 1, CV_AA);
			cv::putText(matchesMat, resultText.str(), cv::Point(image1.cols/2 - 200, image1.rows/2 + 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(10, 10, 255), 1, CV_AA);
			cv::namedWindow("Features", CV_WINDOW_AUTOSIZE);
			cv::imshow("Features", matchesMat);
		}
		info_lastMatch.processingTime = (ros::Time::now().nsec - startTime.nsec)/1000000;
		return true;
	}
	// "else"
	if (this->showWindows) {
		image1.copyTo(matchesMat);
		onScreenText.str(std::string());
		onScreenText << "This is not the wanted object!";
		cv::putText(matchesMat, onScreenText.str(), cv::Point(image1.cols/2 - 200, image1.rows/2), CV_FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(10, 10, 200), 2, CV_AA);
		onScreenText.str(std::string());
		onScreenText << "Ref1  Ref2  Ref3  Ref4  Ref5  Ref6  Ref7  Ref8";
		cv::putText(matchesMat, onScreenText.str(), cv::Point(image1.cols/2 - 200, image1.rows/2 +30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(10, 10, 200), 1, CV_AA);
		cv::putText(matchesMat, resultText.str(), cv::Point(image1.cols/2 - 200, image1.rows/2 + 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(10, 10, 200), 1, CV_AA);
		cv::namedWindow("Features", CV_WINDOW_AUTOSIZE);
		cv::imshow("Features", matchesMat);
	}
	info_lastMatch.processingTime = (ros::Time::now().nsec - startTime.nsec)/1000000;
	return false;
}
