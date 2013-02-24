/*
 * 2013_embedded_worlds.h
 *
 *  Created on: 21.02.2013
 *      Author: andreas
 */

#ifndef EMBEDDED_WORLDS_H_
#define EMBEDDED_WORLDS_H_

#include <imes_vision/RC_feature.h>
#include <youbot_kinematics/youbot_arm_kinematics.hpp>
#include <youbot_showroom/launcher_message.h>

youbot_arm_kinematics* manipulator;
ros::NodeHandle* nh;
ros::Publisher pub;

#endif /* 2013_EMBEDDED_WORLDS_H_ */
