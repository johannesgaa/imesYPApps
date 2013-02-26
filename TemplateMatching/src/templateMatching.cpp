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
#include <ros/package.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include "image_transport/image_transport.h"

#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

#include <youbot_kinematics/youbot_arm_kinematics.hpp>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointAccelerations.h>


#define courseName "TemplateMatching"


cv::Point gVec;
cv::Mat g_template;
brics_actuator::JointPositions last_positions;


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

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch( cv_bridge::Exception& e ) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return -1;
    }

    cv::Mat frame = cv_ptr->image.clone();    cv::Mat result( frame.cols, frame.rows, CV_32FC1 );
	cv::Point matchLoc; double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;

int match_method = CV_TM_SQDIFF_NORMED;

    cv::matchTemplate( frame, g_template, result, match_method);
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ) {
		matchLoc = minLoc;
	} else {
  		matchLoc = maxLoc;
	}

	cv::Point goal(matchLoc.x+(g_template.cols/2),matchLoc.y+(g_template.rows/2));
	cv::Point veloVec(goal.x-frame.cols, goal.y-frame.rows);
	
	// gVec represents the distance in px to the image middle
	gVec = veloVec;

	return 0;
}

void checkAngle(const brics_actuator::JointPositionsConstPtr& joint_pos) {

	if ((int)joint_pos->positions.size() < 5) {
		ROS_WARN("Got less than 5 joint positions!");
	} else {
		//if (joint_pos->positions[0].value < 0.0)
		for (int i=0; i < (int)joint_pos->positions.size(); i++) {
			last_positions.positions[i] = joint_pos->positions[i];
		}
	}
}

int main(int argc, char **argv) {

    /// Init ros part
    ros::init(argc, argv, courseName);

    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber     imageSubscriber;
    imageSubscriber = imageTransport.subscribe("/usb_cam/image_raw", 1, &ImageCallback04);

    ros::Publisher puppy = nodeHandle.advertise<brics_actuator::JointVelocities>("/arm1/joint_controller/joint_velocity_commands", 1);
    ros::Subscriber armsuppy = nodeHandle.subscribe("/arm1/joint_controller/joint_states", 10, &checkAngle);

    std::string path;
    path.clear();
    path = ros::package::getPath("imes_vision");
    path.append("/resources/TQ-logo/1.bmp");
    g_template = cv::imread(path);


   //spin
    while(1) {
    	// 1) do matching
    	ros::spinOnce();

    	// 2) calculate arm_vels relative to gVec (Distances)
    	brics_actuator::JointVelocities cmd_arm_vel;

    	cmd_arm_vel.velocities.resize(5);
    	cmd_arm_vel.velocities[1].value = 0.;
    	cmd_arm_vel.velocities[2].value = 0.;
    	cmd_arm_vel.velocities[3].value = 0.;
    	cmd_arm_vel.velocities[5].value = 0.;

    	// we assume that vel = rad/sec
    	cmd_arm_vel.velocities[0].value = gVec.x/4000;
    	cmd_arm_vel.velocities[4].value = gVec.y/4000;


    	// safety first!
    	if (fabs(cmd_arm_vel.velocities[0].value) > 0.2) {
        	cmd_arm_vel.velocities[0].value = fabs(cmd_arm_vel.velocities[0].value) /(cmd_arm_vel.velocities[0].value) * 0.2;
        	ROS_WARN("Velocity of Joint 0 set to zero!");
    	}
    	if (fabs(cmd_arm_vel.velocities[0].value) > 0.2) {
        	cmd_arm_vel.velocities[4].value = fabs(cmd_arm_vel.velocities[4].value) /(cmd_arm_vel.velocities[4].value) * 0.2;
        	ROS_WARN("Velocity of Joint 0 set to 0.2!");
    	}

    	//check if we are at the end of the joint range:
    	if (last_positions.positions[0].value < 0.5 || last_positions.positions[0].value > 5.5) {
    		cmd_arm_vel.velocities[0].value = 0.;
    		ROS_WARN("End of Range. Will not set vel!");
    	}
    	if (last_positions.positions[4].value < 0.1 || last_positions.positions[4].value > 2.5) {
    		cmd_arm_vel.velocities[4].value = 0.;
    		ROS_WARN("End of Range. Will not set vel!");
    	}

    	puppy.publish(cmd_arm_vel);
    	ros::Duration(0.05).sleep();
     }

    return 0;
}


