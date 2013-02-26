/* *****************************************************************
 *
 * imes_youbot_pkg
 *
 * Copyright (c) 2012,
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
 * @file   controlUnit.hpp
 * @author eduard (eduardpopp@web.de)
 * @date   13.11.2012
 *
 * @brief  Filedescription
 */

#ifndef CONTROLUNIT_HPP
#define CONTROLUNIT_HPP

//ROS includes
#include "ros/ros.h"

#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

//YouBot includes
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointAccelerations.h>
#include <sensor_msgs/JointState.h>
#include <youbot_kinematics/youbot_arm_kinematics.hpp>
#include <geometry_msgs/Twist.h>
#include <youbot_motion_control/motionControl.h>

#include <boost/units/io.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/systems/si.hpp>

class controlUnit {

public:

    controlUnit();

    ~controlUnit();

    ros::Timer timer;
    double timeIt;
    youbot_arm_kinematics* youBotArm;
private:

    std::string configFile;

    /**
      *@brief Callback zum Subsriben der Ist-Positionen und Ist-Geschwindigkeiten
      */
    void JointPositionCallback(const sensor_msgs::JointState& jointPositionCommand);

    /**
      *@brief Callback zur Berechnung der Geschwindigkeiten
      */
    void TimerCallback(const ros::TimerEvent& e);

    /// Subscriber für Positionen und Geschwindigkeiten aus dem JointStates Topic
    ros::Subscriber jointPosVelSubscriber;

    /// Publisher für die Geschwindigkeit an das cmd_velocity Topic
    ros::Publisher jointVelocityPublisher;
    ros::Publisher velocityPublisher;
    ros::Publisher controlerPublisher;

    /// RosTimer

    ros::NodeHandle n;

    /// Messagetyp der Joinstates
    brics_actuator::JointVelocities armJointVelocities;
    brics_actuator::JointPositions armJointPositions;
    brics_actuator::JointPositions controler;


    /// Positionsabweichung
    double e;
    double eSum;
    double F;
    double eAlt;


    /// Macros
    static const unsigned int JOINT1  = 0;
    static const unsigned int JOINT2  = 1;
    static const unsigned int JOINT3  = 2;
    static const unsigned int JOINT4  = 3;
    static const unsigned int JOINT5  = 4;
    static const unsigned int ARMJOINTS = 5;

    static const double MINANGLE1 = 0.576 * M_PI/180;
    static const double MINANGLE2 = 0.576 * M_PI/180;
    static const double MINANGLE3 = -288.0 * M_PI/180;
    static const double MINANGLE4 = 1.266 * M_PI/180;
    static const double MINANGLE5 = 6.337 * M_PI/180;
    static const double MINANGLEGRIPPER = 0.000 * M_PI/180;

    static const double MAXANGLE1 = 334.616 * M_PI/180;
    static const double MAXANGLE2 = 150.0 * M_PI/180;
    static const double MAXANGLE3 = -0.9 * M_PI/180;
    static const double MAXANGLE4 = 196.5 * M_PI/180;
    static const double MAXANGLE5 = 323.241 * M_PI/180;
    static const double MAXANGLEGRIPPER = 0.023 * M_PI/180;

    static const double VMAX1 = 2.685;
    static const double VMAX2 = 2.685;
    static const double VMAX3 = 4.189;
    static const double VMAX4 = 5.900;
    static const double VMAX5 = 5.900;

    static const double AMAX1 = 1.343;
    static const double AMAX2 = 1.343;
    static const double AMAX3 = 2.094;
    static const double AMAX4 = 2.950;
    static const double AMAX5 = 2.950;

    static const long double LOOPRATE = 0.01;

    static const size_t TASKSPACE = 6;
    static const size_t JOINTSPACE = 5;
    static const size_t TRANSLATION = 3;
    static const size_t ORIENTATION = 3;

    //Regler
//    static const double B = 0.;
//    static const double H = 10.;
//    static const double KP = 0.0;
//    static const double KI = 0.00;
//    static const double KD = 0.00;


};

#endif // CONTROLUNIT_HPP
