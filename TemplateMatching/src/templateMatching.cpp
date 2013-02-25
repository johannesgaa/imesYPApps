/* *****************************************************************
 *
 * RobotChallenge Homework
 *
 * Copyright (c) %YEAR%,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/**
 * @file   %FILENAME%
 * @author %USER% (%$EMAIL%)
 * @date   %DATE%
 *
 * @brief  Filedescription
 */



#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <highgui.h>



#define courseName "TemplateMatching"


cv::Point gVec;
cv::Mat g_template;


int ImageCallback04(const sensor_msgs::ImageConstPtr& msg)
{
    /*/
    *
    * Dieser Teil konvertiert das Image in eine Opencv Mat. Will man nicht die Webcam verwenden, sondern
    * einfache Bilder laden, kommentiert man am besten die naechsten Zeilen  aus und fuegt:
    *
    * cv::Mat frame = cv::imread("<PfadZumBild>");
    *darunter ein.
    /*/
    // first we convert the ros message to a opencv image
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch( cv_bridge::Exception& e )
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return -1;
    }
    cv::Mat frame = cv_ptr->image.clone();    cv::Mat result( frame.cols, frame.rows, CV_32FC1 );
	cv::Point matchLoc; double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    matchTemplate( frame, g_template, result, match_method );   normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

 minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )  	{ matchLoc = minLoc; }
  	else { matchLoc = maxLoc; }

	cv::Point goal(matchLoc.x+(g_template.cols/2),matchLoc.y+(g_template.rows/2));
	
	cv::Point veloVec(goal.x-frame.cols, goal.y-frame.rows);
	
	 gVec = veloVec;


	    return 0;

}




int main(int argc, char **argv) {

    /// Init ros part
    ros::init(argc, argv, courseName);

    ros::NodeHandle 				nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber     imageSubscriber;

ros::Publisher puppy = nodeHandle.advertise<brics_actuator::velocity_cmd>("/arm1/joint_command", 1);
puppy.publish();

ros::subscriber armsuppy = nodeHandler.subscribe("/", checkAngle);
    /// subscribe the node to the usbcam topic

    /*/
    *
    * Je nach zu bearbeitende Aufgabe die entsprechende der folgenden drei dekommentieren, damit
    * die richtige Callback Methode aufgerufen wird, siehe Übungszettel.
    *
    /*/
     imageSubscriber = imageTransport.subscribe("/usb_cam/image_raw", 1, &ImageCallback04);
    // imageSubscriber = imageTransport.subscribe("/usb_cam/image_raw", 1, &ImageCallback05);
    // imageSubscriber = imageTransport.subscribe("/usb_cam/image_raw", 1, &ImageCallback06);

  
    g_template = cv::imread("../data/templateTQ.png");


   //spin
    while(1)
    {
	puppy.publish();
        ros::spin();
     }

    return 0;
}


