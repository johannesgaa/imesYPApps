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
 * @file   motionPlanning.cpp
 * @author Eduard Popp (eduardpopp@web.de)
 * @date   26. September 2012
 *
 * @brief  Filedescription
 */

#include <youbot_motion_control/motionPlanning.hpp>

using namespace std;
using namespace Eigen;

motionPlanning::motionPlanning() {
    //Laden der Configfile aus dem Paramterraum
    string path = ros::package::getPath("youbot_driver");
    path.append("/config/youbot-manipulator.cfg");
    youBotArm = new youbot_arm_kinematics(path, "arm_link_0");

    //m_tf = new tf::Transform();

    //Initialisieren der max. Geschwindigkeiten und Beschleunigungen
    Joint.max_acc = AMAX1;    Joint.max_vel = VMAX1;    Joint.minAngle = MINANGLE1;     Joint.maxAngle = MAXANGLE1;     initParamValues.push_back(Joint);
    Joint.max_acc = AMAX2;    Joint.max_vel = VMAX2;    Joint.minAngle = MINANGLE2;     Joint.maxAngle = MAXANGLE2;     initParamValues.push_back(Joint);
    Joint.max_acc = AMAX3;    Joint.max_vel = VMAX3;    Joint.minAngle = MINANGLE3;     Joint.maxAngle = MAXANGLE3;     initParamValues.push_back(Joint);
    Joint.max_acc = AMAX4;    Joint.max_vel = VMAX4;    Joint.minAngle = MINANGLE4;     Joint.maxAngle = MAXANGLE4;     initParamValues.push_back(Joint);
    Joint.max_acc = AMAX5;    Joint.max_vel = VMAX5;    Joint.minAngle = MINANGLE5;     Joint.maxAngle = MAXANGLE5;     initParamValues.push_back(Joint);


    xEndeffector = new double[6];
    ePosSum = new double[JOINTSPACE];
    ePos = new double[JOINTSPACE];
    ePosAlt = new double[JOINTSPACE];
    cartesianToJointInit = new double[ARMJOINTS];
    cartesianToJointGoal = new double[ARMJOINTS];

    BaseCalculation_done = false;
    busy = false;
    velocity_set = false;
    calculation_done = false;
    points = 0;
    timeIt = 0;
    timeBaseIt = 0;

    t_f_base.resize(2);
    KP.resize(5);
    KI.resize(5);
    KD.resize(5);
    windUP.resize(5);
    stellwert.resize(10);

    positionIsReady = false;
    orientationIsReady =false;
    onlyOrientation = false;

    //Initialisierung der Vektoren
    armJointVelocities.velocities.resize(ARMJOINTS);
    armJointPositions.positions.resize(ARMJOINTS);
    currentVelocities.velocities.resize(ARMJOINTS);
    singleAxis.arm_axis.resize(ARMJOINTS);

    regler.positions.resize(15);
    motionController.joint.resize(ARMJOINTS);
    baseParam.resize(3);
    singleWheel.arm_axis.resize(3);



    stringstream jointName;

    for (unsigned int idx = 0; idx < ARMJOINTS; ++idx) {
        jointName.str("");
        jointName << "arm_joint_" << (idx + 1);

        armJointPositions.positions[idx].joint_uri = jointName.str();
        armJointPositions.positions[idx].unit = boost::units::to_string(boost::units::si::radians);
        armJointPositions.positions[idx].value = 0.0;

        currentVelocities.velocities[idx].joint_uri = jointName.str();
        currentVelocities.velocities[idx].unit = boost::units::to_string(boost::units::si::radians_per_second);
        currentVelocities.velocities[idx].value = 0.0;

        armJointVelocities.velocities[idx].joint_uri = jointName.str();
        armJointVelocities.velocities[idx].unit = boost::units::to_string(boost::units::si::radians_per_second);
        armJointVelocities.velocities[idx].value = 0.0;

        ePosSum[idx] = 0.0;
        ePos[idx] = 0.0;
        ePosAlt[idx] = 0.0;
    }
    //Publisher und Subscriber Intialisieren
    timer = n.createTimer(ros::Duration(LOOPRATE), &motionPlanning::TimerCallback, this, false, false);
    timerBase = n.createTimer(ros::Duration(LOOPRATE), &motionPlanning::TimerBaseCallback, this, false, false);
    jointPosVelSubscriber = n.subscribe("/arm_1/joint_states", 1, &motionPlanning::JointPositionCallback, this);
    jointVelocityPublisher = n.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);
    controllerPublisher = n.advertise<youbot_motion_control::motionControl>("/evaluation", 1);
    testPublisher = n.advertise<brics_actuator::JointPositions>("/regler",1);
    basePublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    this->baseSubscriber = this->n.subscribe("/odom", 1, &motionPlanning::baseOdometryCallback, this);

    while(ros::ok() && jointVelocityPublisher.getNumSubscribers()== 0){
        ROS_INFO("Waiting for subscribers on /arm_1/arm_controller/velocity_command");
        ros::Duration(1.0).sleep();
    }

}

motionPlanning::~motionPlanning() {
}

void motionPlanning::init(size_t workspace) {
    DOF = workspace;
    if(DOF == TASKSPACE)
        orientation = 3;
    else
        orientation = 0;

    //clear Time
    t_f_ppoint.clear();

    //clear bools
    velocity_set = false;
    calculation_done = false;
    busy = false;
    ramp_started = false;

    positionIsReady = false;
    orientationIsReady =false;

    //Regler
    KP[0] = 0.08;   KI[0] = 0.00;   KD[0] = 0.000;      windUP[0] = VMAX1;
    KP[1] = 0.1;    KI[1] = 0.00;    KD[0] = 0.000;      windUP[1] = VMAX2;
    KP[2] = 0.07;    KI[2] = 0.000;    KD[0] = 0.000;    windUP[2] = VMAX3;
    KP[3] = 0.125;      KI[3] = 0.0;    KD[0] = 0.000;    windUP[3] = VMAX4;
    KP[4] = 0.1;    KI[4] = 0.0;    KD[0] = 0.000;       windUP[4] = VMAX5;
    GAIN = 5.0;
    fehlerQ = 0.0;
    emax = 0.0;
    emin = 0.0;


    //Iterators
    points = 0;
    timeIt = 0;
    pPoints = 0;

    //clear vectors
    pathParam.clear();
    gripperPoint.clear();
    gripperPosition.clear();
    maxParamValue.clear();
    DiffPath.clear();
    coefficientsSegment.clear();
    coefficients_ppoint.clear();
    timeIncPoint.clear();
    maxJointParamValue.clear();
    pointParam.clear();
    degreeOfFreedom.clear();
    Diff.clear();

    for (size_t k = 0; k < 5; k++)
        xEndeffector[0] = 0.0;


    maxJointParamValue.resize(DOF);
    degreeOfFreedom.resize(6);
    pointParam.resize(6);
    Diff.resize(DOF);

//    timeIncBase.clear();
//    timeIncBaseRot.clear();
    timeIncBaseTrans.clear();
    timeBaseIt = 0;
    startBasePose = lastBasePose;

    baseCoeffs.clear();
}

double motionPlanning::getTurnedAngle(geometry_msgs::Pose _curr_pose) {
    tf::Quaternion _q1(_curr_pose.orientation.x, _curr_pose.orientation.y, _curr_pose.orientation.z, _curr_pose.orientation.w);
    tf::Quaternion _q2(this->startBasePose.orientation.x, this->startBasePose.orientation.y, this->startBasePose.orientation.z, this->startBasePose.orientation.w);

    tf::Quaternion _q3 = _q1.inverse() * _q2;

    return fabs(tf::getYaw(_q3));
}

geometry_msgs::Pose motionPlanning::getTraveledDist(geometry_msgs::Pose _curr_pose) {
    geometry_msgs::Pose tmp;
//    tmp.position.x = _curr_pose.position.x - firstBasePose.position.x;
//    tmp.position.y = _curr_pose.position.y - firstBasePose.position.y;
    tmp.position.x = _curr_pose.position.x - this->startBasePose.position.x;
    tmp.position.y = _curr_pose.position.y - this->startBasePose.position.y;
    return tmp;
}

void motionPlanning::newTrajectory() {
    init(JOINTSPACE);
    setcurrentJointPosition();

}

void motionPlanning::newArmMove() {
    init(TASKSPACE);
    setcurrentTaskPosition();
}

void motionPlanning::JointPositionCallback(const sensor_msgs::JointState &jointPositionCommand) {
    for (size_t m = 0; m < jointPositionCommand.name.size(); m++) {
        for (size_t n = 0; n < armJointPositions.positions.size(); n++) {
            if (armJointPositions.positions[n].joint_uri == jointPositionCommand.name[m]) {
                armJointPositions.positions[n].value = jointPositionCommand.position[m];
                break;
            }
        }
    }
    for (size_t m = 0; m < jointPositionCommand.name.size(); m++) {
        for (size_t n = 0; n < currentVelocities.velocities.size(); n++) {
            if (currentVelocities.velocities[n].joint_uri == jointPositionCommand.name[m]) {
                currentVelocities.velocities[n].value = jointPositionCommand.velocity[m];
                break;
            }

        }
    }//    while(ros::ok() && jointVelocityPublisher.getNumSubscribers()== 0){
    //        ROS_INFO("Waiting for subscribers on /arm_1/arm_controller/velocity_command");
    //        ros::Duration(1.0).sleep();
    //    }
}

void motionPlanning::baseOdometryCallback(const nav_msgs::OdometryConstPtr& odometry_) {
    geometry_msgs::Pose tmp_pose = odometry_->pose.pose;
    //ROS_INFO("x: %f, y: %f, z: %f", tmp_pose.position.x, tmp_pose.position.y, tmp_pose.orientation.w);

    this->lastBasePose = tmp_pose;
}

void motionPlanning::TimerBaseCallback(const ros::TimerEvent &e) {
    if (positionIsReady == false && !onlyOrientation) {
        baseVelocity.linear.x = timeIncBaseTrans[timeBaseIt].arm_axis[0].velocity;
        baseVelocity.linear.y = timeIncBaseTrans[timeBaseIt].arm_axis[1].velocity;
        baseVelocity.angular.z = 0;
        ROS_INFO("base.x: %.3f", timeIncBaseTrans[timeBaseIt].arm_axis[0].position);

//        listener.lookupTransform("base_link", "map", ros::Time(0), transform);
//        secondBasePose.position.x = transform.getOrigin().getX();
//        secondBasePose.position.y = transform.getOrigin().getY();

//        geometry_msgs::Pose dist = getTraveledDist(secondBasePose);
        geometry_msgs::Pose dist = getTraveledDist(this->lastBasePose);
        ROS_INFO("dist.x: %.3f", dist.position.x);
        ePos[0] = timeIncBaseTrans[timeBaseIt].arm_axis[0].position - dist.position.x;
        ePosSum[0] = ePos[0] + ePosSum[0];
        ePos[1] = timeIncBaseTrans[timeBaseIt].arm_axis[1].position - dist.position.y;
        ePosSum[1] = ePos[1] + ePosSum[1];
        ROS_INFO("e.x: %.3f", ePos[0]);
        ePos[1] = timeIncBaseTrans[timeBaseIt].arm_axis[1].position - dist.position.y;
        ROS_INFO("e.y: %.3f", ePos[1]);
            baseVelocity.linear.x = 5.0*(ePos[0] + 0.25*LOOPRATE*ePosSum[0]) + baseVelocity.linear.x;

            baseVelocity.linear.y = 5.0*(ePos[1] + 0.25*LOOPRATE*ePosSum[1]) + baseVelocity.linear.y;
            regler.positions[9].value = dist.position.y;
            regler.positions[10].value = timeIncBaseTrans[timeBaseIt].arm_axis[1].position;
            regler.positions[11].value = ePosSum[1];
            testPublisher.publish(regler);
            if (ePosSum[0] > GAIN)
                ePosSum[0] = ePosSum[0] - 2*(ePosSum[0] - GAIN);
            if (ePosSum[0] < (-1)*GAIN)
                ePosSum[0] = ePosSum[0] - 2*(ePosSum[0] + GAIN);



    }

    if (positionIsReady == true) {
        baseVelocity.linear.x = 0;
        baseVelocity.linear.y = 0;
        baseVelocity.angular.z = timeIncBaseRot[timeBaseIt].arm_axis[0].velocity;
        ROS_INFO("phi: %.f", baseVelocity.angular.z);

        double angle = getTurnedAngle(this->lastBasePose);

//        ROS_INFO("gedrehter Winkel: %.2f, X-R: %.2f, Y-R: %.2f", angle*180/M_PI, dist.position.x, dist.position.y);
        ROS_INFO("SollPosition = %.2f", timeIncBaseRot[timeBaseIt].arm_axis[0].position);



        ePos[2] = timeIncBaseRot[timeBaseIt].arm_axis[0].position  - angle ;
        ePosSum[2] = ePos[2] + ePosSum[2];
        ROS_INFO("e = %.3f", ePos[2]);
        baseVelocity.angular.z = 5.0*(ePos[2] + 0.5*LOOPRATE*ePosSum[2]) + baseVelocity.angular.z;

        regler.positions[9].value = angle;
        regler.positions[10].value = timeIncBaseRot[timeBaseIt].arm_axis[0].position;
        regler.positions[11].value = ePosSum[2];
        testPublisher.publish(regler);
        string path = "/home/eduard/ros_stacks/imes_youbot_pkg/youbot_motion_control/test/logfiles/Vorsteuerung/controller_base_z";
        stringstream ss;
        ss << path << KP[0] << "_" << KI[0]<< "geregelt.txt";
        string file;
        ss >> file;
        char * buffer = new char[file.length()];
        strcpy(buffer,file.c_str());
        fstream f;
        f.open(buffer, ios::app | ios::out);
        f << timeBaseIt*LOOPRATE << "\t  " << regler.positions[9].value << "\t  " << regler.positions[10].value
          << "\t  "<< regler.positions[5].value << "\t  " << regler.positions[8].value << "\t  " <<  endl;
        f.close();
    }

//    ROS_INFO("eX = %.2f, ePhi = %.2f", ePos[0], ePos[2]);




    basePublisher.publish(baseVelocity);
    timeBaseIt++;

    if (timeBaseIt >= timeIncBaseTrans.size() && !positionIsReady) {
        timeBaseIt = 0;
        positionIsReady = true;
    }

    if (timeBaseIt >= timeIncBaseRot.size() && positionIsReady && !orientationIsReady) {
        timeBaseIt = 0;
        orientationIsReady = true;
    }


    if (positionIsReady && orientationIsReady) {
        baseVelocity.linear.x = 0.0;
        baseVelocity.linear.y = 0.0;
        baseVelocity.angular.z = 0.0;

        basePublisher.publish(baseVelocity);
        timerBase.stop();
        busy = false;
        ROS_INFO("Timer aus!");
//        exit(1);
    }
}


void motionPlanning::TimerCallback(const ros::TimerEvent &ex) {

    for (size_t k = 0; k < ARMJOINTS; k++) {
        armJointVelocities.velocities[k].value = timeIncPoint[timeIt].arm_axis[k].velocity;

        ROS_INFO("Ausführung");
        motionController.joint[k].position = timeIncPoint[timeIt].arm_axis[k].position;
        motionController.joint[k].velocity = timeIncPoint[timeIt].arm_axis[k].velocity;
        motionController.joint[k].acceleration = timeIncPoint[timeIt].arm_axis[k].acceleration;
        //motionController.joint[k].jerk = timeIncPoint[timeIt].arm_axis[k].jerk;

        //---------------------------------------------Regler-----------------------------------------------------------//

        ePos[k] = timeIncPoint[timeIt].arm_axis[k].position * 180/M_PI - armJointPositions.positions[k].value * 180/M_PI;
        //ePosSum[k] = ePos[k] + ePosSum[k];
        stellwert[k] = KP[k]*(ePos[k]  + KI[k]*ePosSum[k]*LOOPRATE);// + KD[k]*(ePos[k] - ePosAlt[k])/LOOPRATE)
//        armJointVelocities.velocities[k].value = armJointVelocities.velocities[k].value + KP[k]*(ePos[k]);//  + KI[k]*ePosSum[k]*LOOPRATE);
//        if (armJointVelocities.velocities[k].value > windUP[k])
//            armJointVelocities.velocities[k].value = windUP[k];
//        if (armJointVelocities.velocities[k].value < (-1)*windUP[k])
//            armJointVelocities.velocities[k].value = (-1)*windUP[k];

        fehlerQ = fehlerQ + pow(ePos[TESTJOINT],2);
        //double F = armJointVelocities.velocities[k].value;
//        //Anti Windup
//        if (armJointVelocities.velocities[k].value > windUP[k])
//            ePosSum[k] = ePosSum[k] - GAIN*(stellwert[k] - windUP[k]);
//        if (armJointVelocities.velocities[k].value < (-1)*windUP[k])
//            ePosSum[k] = ePosSum[k] - GAIN*(stellwert[k] + windUP[k]);
        regler.positions[k].value = ePos[k];
        //ePosAlt[k] = ePos[k];
        //---------------------------------------------------------------------------------------------------------------//

    }
    regler.positions[7].value = timeIncPoint[timeIt].arm_axis[TESTJOINT].position * 180/M_PI;
    regler.positions[6].value = armJointPositions.positions[TESTJOINT].value * 180/M_PI;
    regler.positions[5].value = armJointVelocities.velocities[0].value;
    //regler.positions[9].value = ePosSum[TESTJOINT];
    //regler.positions[10].value = stellwert[TESTJOINT];
    regler.positions[8].value  = currentVelocities.velocities[TESTJOINT].value;
    //Messwerte aufzeichnen
    string path = "/home/eduard/ros_stacks/imes_youbot_pkg/youbot_motion_control/test/logfiles/Vorsteuerung/controller_";
    stringstream ss;
    ss << path << KP[0] << "_" << "ungeregelt.txt";
    string file;
    ss >> file;
    char * buffer = new char[file.length()];
    strcpy(buffer,file.c_str());
    fstream f;
    f.open(buffer, ios::app | ios::out);
    f << timeIt*LOOPRATE << "\t  " << regler.positions[7].value << "\t  " << regler.positions[6].value
      << "\t  "<< regler.positions[5].value << "\t  " << regler.positions[8].value << "\t  " <<  endl;
    f.close();
//    fehlerQ = fehlerQ + pow(ePos[TESTJOINT],2);

    jointVelocityPublisher.publish(armJointVelocities);
    //controllerPublisher.publish(motionController);
    testPublisher.publish(regler);

//    if (ePos[TESTJOINT] > emax)
//        emax = ePos[TESTJOINT];
//    if (ePos[TESTJOINT] < emin)
//        emin = ePos[TESTJOINT];

    jointVelocityPublisher.publish(armJointVelocities);
    for(size_t gripperIt = 0; gripperIt < gripperPoint.size() ; gripperIt++) {
        if(gripperPoint[gripperIt] == timeIt) {
            youBotArm->setGripper(gripperPosition[gripperIt]);
            moveGripper();
        }
    }
    timeIt++;

    // Geschwindigkeit zur Sicherheit am Ende der Bewegung auf Null setzen und Timer anhalten
    if (timeIt >= timeIncPoint.size()) {
        for (size_t k = 0; k < ARMJOINTS; k++) {
            armJointVelocities.velocities[k].value = 0.0;
        }
        jointVelocityPublisher.publish(armJointVelocities);
        timer.stop();
        //ROS_INFO("Summer FehlerQ: %i, %i \n %.1f, %.1f, %.3f", (int)fehlerQ, (int)sqrt(fehlerQ), emax, emin, ePos[TESTJOINT]);

        busy = false;

        //youBotArm->setJntPos(timeIncPoint[timeIt-1].arm_axis[0].position, timeIncPoint[timeIt-1].arm_axis[1].position,
        //                     timeIncPoint[timeIt-1].arm_axis[2].position, timeIncPoint[timeIt-1].arm_axis[3].position,
        //                     timeIncPoint[timeIt-1].arm_axis[4].position);
        //youBotArm->moveToPose();

    }

    /*
    //ROS_INFO("timerCallback working!");

    if(timercallback) {
        if (busy==false) {
            l_temp = 0;
            t = 0.0;
            busy = true;
            gettimeofday(&detail_time,NULL);
            start_time = double(detail_time.tv_sec + 0.000001*detail_time.tv_usec);
            filter_idx = 0;
        }

        if (l_temp < (int)points) {
            gettimeofday(&new_time,NULL);
            current_time = double(new_time.tv_sec + 0.000001*new_time.tv_usec);
            t = current_time - start_time;
            if (t < t_f_ppoint[l_temp]) {
                if (gripperSet == (int)l_temp+1)
                    moveGripper();

                for (size_t k = 0; k < ARMJOINTS; k++) {
                    // Bestimmung der Gelenkwinkelgeschwindkeit
                    armJointVelocities.velocities[k].value = (1*coefficients_ppoint[l_temp][k].a1 + 2*coefficients_ppoint[l_temp][k].a2*t +
                                            3*coefficients_ppoint[l_temp][k].a3*pow(t,2) + 4*coefficients_ppoint[l_temp][k].a4*pow(t,3) +
                                                              5*coefficients_ppoint[l_temp][k].a5*pow(t,4));
                    // Bestimmung der Gelenkwinkelposition
                    planned_pos[k] = (1*coefficients_ppoint[l_temp][k].a0 + 1*coefficients_ppoint[l_temp][k].a1*t +
                                                1*coefficients_ppoint[l_temp][k].a2*pow(t,2) + 1*coefficients_ppoint[l_temp][k].a3*pow(t,3) +
                                                1*coefficients_ppoint[l_temp][k].a4*pow(t,4) + 1*coefficients_ppoint[l_temp][k].a5*pow(t,5));

                    ePos[k] = planned_pos[k]*180/M_PI - armJointPositions.positions[k].value*180/M_PI;
                    abweichung.positions[k].value = ePos[k];

                    ePosSum[k] = ePos[k] + ePosSum[k];

                    //Anti Windup
                    if (ePosSum[k] > 10.0)
                        ePosSum[k] = ePosSum[k] - 2*(ePosSum[k] - 10.0);
                    if (ePosSum[k] < -10.0)
                        ePosSum[k] = ePosSum[k] - 2*(ePosSum[k] + 10.0);

                    armJointVelocities.velocities[k].value = 0.05 * ePos[k] + 0.007 * ePosSum[k] + 0.0*ediff[k] +
                                                             armJointVelocities.velocities[k].value;

                }
                differencePublisher.publish(abweichung);
                //checkJointLimit();
                jointVelocityPublisher.publish(armJointVelocities);
                //ROS_INFO("Velocity_setting!");
                double P[5], V[5];
                getJointVelocity(V);
                getJointPosition(P);
                ROS_INFO("Ges = %.3f", V[3]);
                ROS_INFO("Pos = %.3f", P[3]);

            }
            else {
                gettimeofday(&detail_time,NULL);
                start_time = double(detail_time.tv_sec + 0.000001*detail_time.tv_usec);
                l_temp++;
                t = 0.0;
            }
        }
        else {
            for (size_t k = 0; k < ARMJOINTS; k++) {
                armJointVelocities.velocities[k].value = 0.0;
            }
            jointVelocityPublisher.publish(armJointVelocities);
            init();
            my_init();
            ROS_INFO("Motion done!");
            timercallback = false;
        }
    }
    */
}

void motionPlanning::setcurrentJointPosition() {
    for (size_t k = 0; k < JOINTSPACE; k++)
        pointParam[k].position = armJointPositions.positions[k].value;

    pathParam.push_back(pointParam);
}

void motionPlanning::setcurrentTaskPosition() {
    youBotArm->getCartEEPose(xEndeffector[0], xEndeffector[1], xEndeffector[2], xEndeffector[3], xEndeffector[4], xEndeffector[5]);
    getJointPosition(cartesianToJointInit);

    jntTwoParamInit.position = cartesianToJointInit[1];
    jntTwoParamInit.velocity = 0.0;
    jntTwoParamInit.acceleration = 0.0;

    pointParam[0].position = xEndeffector[0];
    pointParam[1].position = xEndeffector[1];
    pointParam[2].position = xEndeffector[2];
    pointParam[3].position = xEndeffector[3];
    pointParam[4].position = xEndeffector[4];
    pointParam[5].position = xEndeffector[5];
    pathParam.push_back(pointParam);
}

void motionPlanning::moveArm(double _x, double _y, double _z, double _roll, double _pitch) {

    //youBotArm->getIK(cartesianToJoint, xEndeffector[0], xEndeffector[1], xEndeffector[2], xEndeffector[3], xEndeffector[4], xEndeffector[5], getOverhead(xEndeffector[0],xEndeffector[1]));
    //q2Start = q_temp[1];
    //q1 = q_temp[0];

    if (!velocity_set)
        setManipulatorVelocity(0.05);

    xEndeffector[0] += _x;
    xEndeffector[1] += _y;
    xEndeffector[2] += _z;
    xEndeffector[3] += _roll;
    xEndeffector[4] += _pitch;
    //xEndeffector[5] += _yaw;

    // Überprüfung der neuen Koordinaten
    youBotArm->getIK(cartesianToJointGoal, xEndeffector[0], xEndeffector[1], xEndeffector[2], xEndeffector[3], xEndeffector[4], xEndeffector[5], false);// getOverhead(xEndeffector[0],xEndeffector[1]));
    //q2End = q_temp[1];
    checkPositionRange(cartesianToJointGoal[0], cartesianToJointGoal[1], cartesianToJointGoal[2], cartesianToJointGoal[3], cartesianToJointGoal[4]);

    jntTwoParamGoal.position = cartesianToJointGoal[1];
    jntTwoParamGoal.velocity = 0.0;
    jntTwoParamGoal.acceleration = 0.0;

    pointParam[0].position = xEndeffector[0];
    pointParam[1].position = xEndeffector[1];
    pointParam[2].position = xEndeffector[2];
    pointParam[3].position = xEndeffector[3];
    pointParam[4].position = xEndeffector[4];
    pointParam[5].position = xEndeffector[5];

    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;
}

bool motionPlanning::getOverhead(double _x, double _y) {
    // 1 Zwangsbedingung: x > 0 ---> links von der y-Achse
    // 2. Zwangsbedingung: 0.1994*x + y > 0 ---> oberhalb der 0°-Geraden von Joint1
    // 3. Zwangsbedingung: 0.1994*x - y > 0 ---> unterhalb der 338°-Geraden von Joint1
    if(_x > 0.0 && (0,1994*_x + _y) > 0.0 && (0,1994*_x - _y) > 0.0)
            return false;
    else
            return true;
}

void motionPlanning::setManipulatorVelocity(double manipulatorVel) {
    for (size_t k = 0; k < TASKSPACE; k++) {
        maxJointParamValue[k].max_vel = manipulatorVel;
        maxJointParamValue[k].max_acc = 0.5;
    }
    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}

void motionPlanning::setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5) {
    checkPositionRange(JointPos1, JointPos2, JointPos3, JointPos4, JointPos5);

    if (!velocity_set)
        setTargetJointVelocity(1.0);

    pointParam[0].position = JointPos1;
    pointParam[1].position = JointPos2;
    pointParam[2].position = JointPos3;
    pointParam[3].position = JointPos4;
    pointParam[4].position = JointPos5;

    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;
}

void motionPlanning::setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5, double gripperPos) {
    checkPositionRange(JointPos1, JointPos2, JointPos3, JointPos4, JointPos5);

    if (!velocity_set)
        setTargetJointVelocity(1.0);

    pointParam[0].position = JointPos1;
    pointParam[1].position = JointPos2;
    pointParam[2].position = JointPos3;
    pointParam[3].position = JointPos4;
    pointParam[4].position = JointPos5;
    gripperPosition.push_back(gripperPos);
    gripperPoint.push_back(points);

    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;
}

void motionPlanning::setPathPoint(double JointPos[5]) {
    checkPositionRange(JointPos[0], JointPos[1], JointPos[2], JointPos[3], JointPos[4]);

    if (!velocity_set)
        setTargetJointVelocity(1.0);

    for (size_t k = 0; k < ARMJOINTS; k++) {
        pointParam[k].position = JointPos[k];
    }
    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;
}

void motionPlanning::checkPositionRange(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5) {
    if ((JointPos1 < initParamValues[JOINT1].minAngle) || (JointPos1 > initParamValues[JOINT1].maxAngle)) {
        ROS_ERROR("Jointposition 1 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos1);
        exit(1);
    }
    else if ((JointPos2 < initParamValues[JOINT2].minAngle) || (JointPos2 > initParamValues[JOINT2].maxAngle)) {
        ROS_ERROR("Jointposition 2 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos2);
        exit(1);
    }
    else if ((JointPos3 < initParamValues[JOINT3].minAngle) || (JointPos3 > initParamValues[JOINT3].maxAngle)) {
        ROS_ERROR("Jointposition 3 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos3);
        exit(1);
    }
    else if ((JointPos4 < initParamValues[JOINT4].minAngle) || (JointPos4 > initParamValues[JOINT4].maxAngle)) {
        ROS_ERROR("Jointposition 4 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos4);
        exit(1);
    }
    else if ((JointPos5 < initParamValues[JOINT5].minAngle) || (JointPos5 > initParamValues[JOINT5].maxAngle)) {
        ROS_ERROR("Jointposition 5  of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos5);
        exit(1);
    }
}

void motionPlanning::setTargetJointVelocity(double JointVel1, double JointVel2, double JointVel3, double JointVel4, double JointVel5) {
    if (JointVel1 < initParamValues[JOINT1].max_vel) {
        maxJointParamValue[JOINT1].max_vel = JointVel1;
        maxJointParamValue[JOINT1].max_acc = initParamValues[JOINT1].max_acc;
    }
    if (JointVel2 < initParamValues[JOINT2].max_vel) {
        maxJointParamValue[JOINT2].max_vel = JointVel2;
        maxJointParamValue[JOINT2].max_acc = initParamValues[JOINT2].max_acc;
    }
    if (JointVel3 < initParamValues[JOINT3].max_vel) {
        maxJointParamValue[JOINT3].max_vel = JointVel3;
        maxJointParamValue[JOINT3].max_acc = initParamValues[JOINT3].max_acc;
    }
    if (JointVel4 < initParamValues[JOINT4].max_vel) {
        maxJointParamValue[JOINT4].max_vel = JointVel4;
        maxJointParamValue[JOINT4].max_acc = initParamValues[JOINT4].max_acc;
    }
    if (JointVel5 < initParamValues[JOINT5].max_vel) {
        maxJointParamValue[JOINT5].max_vel = JointVel5;
        maxJointParamValue[JOINT5].max_acc = initParamValues[JOINT5].max_acc;
    }

    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}

void motionPlanning::setTargetJointVelocity(vector <double> JointVel) {
    if (JointVel.size() != ARMJOINTS)
        ROS_ERROR("setTargetJointVelocity expects 5 Velocities. %d are given", (int)JointVel.size());
    for (size_t k = 0; k < ARMJOINTS; k++) {
        maxJointParamValue[k].max_acc = initParamValues[k].max_acc;
        if (JointVel[k] < initParamValues[k].max_vel)
            maxJointParamValue[k].max_vel = JointVel[k];
        else
            maxJointParamValue[k].max_vel = initParamValues[k].max_vel;
    }
    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}

void motionPlanning::setTargetJointVelocity(double velfactor) {
    if (velfactor < 0.0) {
        ROS_ERROR("Velocity musst be bigger than 0");
        exit(1);
    }

    for (size_t k = 0; k < ARMJOINTS; k++) {
        maxJointParamValue[k].max_vel = initParamValues[k].max_vel*velfactor;
        maxJointParamValue[k].max_acc = initParamValues[k].max_acc;
    }

    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}

void motionPlanning::getJointVelocity(double JointVel[ARMJOINTS]) {
    for (size_t k = 0; k < ARMJOINTS; k++) {
        JointVel[k] = currentVelocities.velocities[k].value;
    }
}

void motionPlanning::getJointVelocity(int Joint, double &JointVel) {
    JointVel = currentVelocities.velocities[Joint].value;
}

void motionPlanning::getJointPosition(double JointPos[5]) {
    for (size_t k = 0; k < ARMJOINTS; k++) {
        JointPos[k] = armJointPositions.positions[k].value;
    }
}

void motionPlanning::getJointPosition(int Joint, double &JointPos) {
    JointPos = armJointPositions.positions[Joint].value;
}

void motionPlanning::calculatePathProperties() {
    double t_vmax = 0.0;
    double t_amax = 0.0;
    double t_max = 0.0;
    double t_f = 0.0;
    double t_Orient = 0.0;
    COEFFS coefficientsJoint;

    //Diff = q_z - q-s
    for (size_t l = 0; l < points; l++) {
        for (size_t k = 0; k < DOF; k++) {
            Diff[k] = pathParam[l+1][k].position - pathParam[l][k].position;
        }
        DiffPath.push_back(Diff);
    }

    //Berechnen der Maximalen Trajektorierenzeit auf Basis der Maximalen Geschwindikeiten/Beschleunigungen
    for (size_t l = 0; l < points; l++) {
        if (DOF == TASKSPACE) {
            // Translation
            for (size_t k = 0; k < (TRANSLATION); k++) {
                t_vmax = 15.0/8.0*fabs(DiffPath[l][k])/maxParamValue[l][k].max_vel;
                t_amax = sqrt(10.0/sqrt(3.0)*fabs(DiffPath[l][k])/maxParamValue[l][k].max_acc);
                t_max = max(t_vmax, t_amax);
                if (t_max > t_f)
                    t_f = t_max;
            }
            // Orientierung
            for (size_t k = (ORIENTATION); k < TASKSPACE; k++) {
                t_Orient = fabs(DiffPath[l][k])/maxParamValue[l][k].max_vel;
                if (t_Orient > t_f)
                    t_f = t_Orient;
            }
        }
        else {
            // Jointspace
            for (size_t k = 0; k < JOINTSPACE; k++) {
                t_vmax = 15.0/8.0*fabs(DiffPath[l][k])/maxParamValue[l][k].max_vel;
                t_amax = sqrt(10.0/sqrt(3.0)*fabs(DiffPath[l][k])/maxParamValue[l][k].max_acc);
                t_max = max(t_vmax, t_amax);
                if (t_max > t_f)
                    t_f = t_max;
            }
        }
        t_f_ppoint.push_back(t_f);
        t_f = 0.0;
    }

    //Geschwindigkeiten und Beschleunigungen setzen
    for (size_t l = 0; l <= points; l++)	{
        for (size_t k = 0; k < (DOF - orientation); k++) {
            if(l == 0) {
                pathParam[l][k].acceleration = 0.0;
                pathParam[l][k].velocity = 0.0;
            }
            else if(l == points) {
                pathParam[l][k].velocity = 0.0;
                pathParam[l][k].acceleration = 0.0;
            }
            else {
                if(pathParam[l-1][k].position > pathParam[l][k].position &&
                   pathParam[l][k].position < pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = 0.0;
                    pathParam[l][k].acceleration = sqrt(10/sqrt(3)*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1]);
                }
                else if(pathParam[l-1][k].position > pathParam[l][k].position &&
                        pathParam[l][k].position > pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = -10/8*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1];
                    pathParam[l][k].acceleration = 0.0;
                }
                else if(pathParam[l-1][k].position < pathParam[l][k].position &&
                        pathParam[l][k].position < pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = 10/8*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1];
                    pathParam[l][k].acceleration = 0.0;
                }
                else if(pathParam[l-1][k].position < pathParam[l][k].position &&
                        pathParam[l][k].position > pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = 0.0;
                    pathParam[l][k].acceleration = -sqrt(10/sqrt(3)*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1]);
                }
                else
                {
                    pathParam[l][k].velocity = 0.0;
                    pathParam[l][k].acceleration = 0.0;
                }
            }
        }
    }

    //Interpolation der relativen Zeit für SLERP
    if (DOF == TASKSPACE) {
        // Interpolation von pitch
        setPolynomCoefficients(pathParam[0][4], pathParam[1][4], t_f_ppoint[0], pitchCoeff);
        // Interpolation von roll
        setPolynomCoefficients(pathParam[0][3], pathParam[1][3], t_f_ppoint[0], rollCoeff);
        // InterPolation von Jnt2
        setPolynomCoefficients(jntTwoParamInit, jntTwoParamGoal, t_f_ppoint[0], jntTwoCoeff);
    }

    //Bestimmung der Koeffizienten
    for (size_t l = 0; l < points; l++) {
        for (size_t k = 0; k < (DOF - orientation); k++) {
            setPolynomCoefficients(pathParam[l][k], pathParam[l+1][k], t_f_ppoint[l], coefficientsJoint);
            coefficientsSegment.push_back(coefficientsJoint);
        }
        coefficients_ppoint.push_back(coefficientsSegment);
        coefficientsSegment.clear();
    }
    calculation_done = true;
}

void motionPlanning::setPolynomCoefficients(PARAM _paramInit, PARAM _paramGoal, double _trajecTime, COEFFS &_coefficient) {

    _coefficient.a0 = _paramInit.position;
    _coefficient.a1 = _paramInit.velocity;
    _coefficient.a2 = _paramInit.acceleration/2;
    _coefficient.a3 = (20*(_paramGoal.position - _paramInit.position) -
                       (8*_paramGoal.velocity + 12*_paramInit.velocity)*_trajecTime -
                       (3*_paramInit.acceleration - _paramGoal.acceleration)*pow(_trajecTime,2)) / (2*pow(_trajecTime,3));
    _coefficient.a4 = (30*(_paramInit.position - _paramGoal.position) +
                       (14*_paramGoal.velocity + 16*_paramInit.velocity)*_trajecTime +
                       (3*_paramInit.acceleration - 2*_paramGoal.acceleration)*pow(_trajecTime,2)) / (2*pow(_trajecTime,4));
    _coefficient.a5 = (12*(_paramGoal.position - _paramInit.position) -
                       6*(_paramGoal.velocity + _paramInit.velocity)*_trajecTime -
                       (_paramInit.acceleration - _paramGoal.acceleration)*pow(_trajecTime,2)) / (2*pow(_trajecTime,5));

}

void motionPlanning::offlineMotionPlanning() {
    double xDot[6];
    double qDot[5];
    double q[5];
    double Jinv[5][5];
    double Jacobi[5][5];
    double wrappedTime = 0.0;
    //double pitch, pitchDot;
    //double q2Temp, q2DotTemp;
//    Vector3d omega;
//    tf::Quaternion quatStart;
//    tf::Quaternion quatEnd;
    ROS_INFO("Planung");
    calculatePathProperties();

    for (size_t l = 0; l < points; l++) {
        wrappedTime = 0.0;

        for (size_t gripperIt = 0; gripperIt < gripperPoint.size(); gripperIt++) {
            if (gripperPoint[gripperIt] == l)
                gripperPoint[gripperIt] = timeIncPoint.size();
        }

        while (wrappedTime < t_f_ppoint[l])  {
             for (size_t k = 0; k < (DOF - orientation); k++) {
                 setPolynomFunction(coefficients_ppoint[l][k], wrappedTime, degreeOfFreedom[k]);
                             //12*coefficients_ppoint[l][k].a4*pow(wrappedTime,2) + 20*coefficients_ppoint[l][k].a5*pow(wrappedTime,3);

                if(DOF == TASKSPACE)
                    xDot[k] = degreeOfFreedom[k].velocity;
                else {
                    singleAxis.arm_axis[k].position = degreeOfFreedom[k].position;
                    singleAxis.arm_axis[k].velocity = degreeOfFreedom[k].velocity;
                    singleAxis.arm_axis[k].acceleration = degreeOfFreedom[k].acceleration;
                }
            }

            if(DOF == TASKSPACE) {


//                interpOrient(wrappedTime, xEndeffector[0], xEndeffector[2]);


//                if (wrappedTime == 0) {
//                    quatStart.setEulerZYX(pathParam[l][3].position, pathParam[l][4].position, pathParam[l][5].position);
//                    quatEnd.setEulerZYX(pathParam[l+1][3].position, pathParam[l+1][4].position, pathParam[l+1][5].position);
//                }
//                getOmega(quatStart, quatEnd, wrappedTime, omega);
//                xDot[3] = omega(0);
//                xDot[4] = omega(1);
//                xDot[5] = omega(2);

                youBotArm->getIK(q, degreeOfFreedom[0].position, degreeOfFreedom[1].position, degreeOfFreedom[2].position,
                                 degreeOfFreedom[3].position, degreeOfFreedom[4].position, degreeOfFreedom[5].position, false);//getOverhead(degreeOfFreedom[0].position, degreeOfFreedom[1].position));
                JACO(q, Jacobi);
                invA(Jacobi, Jinv);
                getqDot(Jinv, xDot, qDot);
                for (size_t k = 0; k < ARMJOINTS; k++) {
                    singleAxis.arm_axis[k].velocity = qDot[k];
                    singleAxis.arm_axis[k].position = q[k];
                }

            }
            wrappedTime = wrappedTime + LOOPRATE;
            //timeIncPoint.push_back(singleAxis);
            timeIncPoint.push_back(singleAxis);
        }
    }
}

void motionPlanning::setPolynomFunction(COEFFS _coeff, double _time, PARAM &_param) {
    _param.position = 1*_coeff.a0 + 1*_coeff.a1*_time + 1*_coeff.a2*pow(_time,2) + 1*_coeff.a3*pow(_time,3) +
                      1*_coeff.a4*pow(_time,4) + 1*_coeff.a5*pow(_time,5);

    _param.velocity = 1*_coeff.a1 + 2*_coeff.a2*_time + 3*_coeff.a3*pow(_time,2) + 4*_coeff.a4*pow(_time,3) +
                      5*_coeff.a5*pow(_time,4);

    _param.acceleration = 2*_coeff.a2 + 6*_coeff.a3*_time + 12*_coeff.a4*pow(_time,2) + 20*_coeff.a5*pow(_time,3);

}

void motionPlanning::interpOrient(double _t, double _x, double _z) {
    double a1 = 0.033, d1 = 0.147, a2 = 0.155, d5 = 0.218;
    double temp1, temp2;
    //double jnt3Function, q3MinusFunction, q3Dot, jnt4Function, jnt4MinusFunction, q4Dot;
    //double jntTwoTemp, jntTwoDotTemp, pitchTemp, pitchDotTemp;
    PARAM jnt2Function, jnt3Function, jnt4Function, jnt5Function, pitchFunction;
    PARAM pitchTemp, jnt2Temp, jnt3Temp, jnt4Temp;

    setPolynomFunction(rollCoeff, _t, jnt5Function);
    setPolynomFunction(jntTwoCoeff, _t, jnt2Function);
    setPolynomFunction(pitchCoeff, _t, pitchFunction);

    temp1 = _x/cos(cartesianToJointInit[0]) - a1 - sin(jnt2Function.position)*a2 - cos(pitchFunction.position)*d5;
    temp2 = _z - d1 - cos(jnt2Function.position)*a2 - sin(pitchFunction.position)*d5;
    jnt3Function.position = atan2(temp1, temp2) - jnt2Function.position;
    jnt4Function.position = M_PI/2 - pitchFunction.position - jnt2Function.position - jnt3Function.position;

    if (_t == 0.0)
        _t = 0.0;
    else
        _t -= LOOPRATE;

    setPolynomFunction(jntTwoCoeff, _t, jnt2Temp);
    setPolynomFunction(pitchCoeff, _t, pitchTemp);

    temp1 = _x/cos(cartesianToJointInit[0]) - a1 - sin(jnt2Temp.position)*a2 - cos(pitchTemp.position)*d5;
    temp2 = _z - d1 - cos(jnt2Temp.position)*a2 - sin(pitchTemp.position)*d5;

    jnt3Temp.position = atan2(temp1, temp2) - jnt2Temp.position;
    jnt4Temp.position = M_PI/2 - pitchTemp.position - jnt2Temp.position - jnt3Temp.position;

    jnt3Function.velocity = (jnt3Function.position - jnt3Temp.position)/LOOPRATE;
    jnt4Function.velocity = -(jnt4Function.position - jnt4Temp.position)/LOOPRATE;

    //singleAxis.arm_axis[0] = 0.0;
    singleAxis.arm_axis[1] = jnt2Function;
    singleAxis.arm_axis[2] = jnt3Function;
    singleAxis.arm_axis[3] = jnt4Function;
    singleAxis.arm_axis[4] = jnt5Function;

//    singleAxis.arm_axis[0].position = 1*roll_X.a0 + 1*roll_X.a1*_t + 1*roll_X.a2*pow(_t,2) +
//           1*roll_X.a3*pow(_t,3) + 1*roll_X.a4*pow(_t,4) + 1*roll_X.a5*pow(_t,5);
//    singleAxis.arm_axis[0].velocity = 1*roll_X.a1 + 2*roll_X.a2*_t + 3*roll_X.a3*pow(_t,2) +
//              4*roll_X.a4*pow(_t,3) + 5*roll_X.a5*pow(_t,4);

//    jntTwoTemp = 1*jntTwo.a0 + 1*jntTwo.a1*_t + 1*jntTwo.a2*pow(_t,2) +
//           1*jntTwo.a3*pow(_t,3) + 1*jntTwo.a4*pow(_t,4) + 1*jntTwo.a5*pow(_t,5);
//    jntTwoDotTemp = 1*jntTwo.a1 + 2*jntTwo.a2*_t + 3*jntTwo.a3*pow(_t,2) +
//              4*jntTwo.a4*pow(_t,3) + 5*jntTwo.a5*pow(_t,4);

//    pitchTemp = 1*orientCo.a0 + 1*orientCo.a1*_t + 1*orientCo.a2*pow(_t,2) +
//           1*orientCo.a3*pow(_t,3) + 1*orientCo.a4*pow(_t,4) + 1*orientCo.a5*pow(_t,5);
//    pitchDotTemp = 1*orientCo.a1 + 2*orientCo.a2*_t + 3*orientCo.a3*pow(_t,2) +
//              4*orientCo.a4*pow(_t,3) + 5*orientCo.a5*pow(_t,4);

//    singleAxis.arm_axis[1].position = jntTwoTemp;
//    singleAxis.arm_axis[1].velocity = jntTwoDotTemp;

//    temp1 = _x/cos(q1) - a1 - sin(jntTwoTemp)*a2 - cos(pitchTemp)*d5;
//    temp2 = _z - d1 - cos(jntTwoTemp)*a2 - sin(pitchTemp)*d5;
//    temp = temp1/temp2;
//    q3 = atan2(temp1, temp2) - jntTwoTemp;
//    q4 = M_PI/2 - pitchTemp - jntTwoTemp - q3;
//    singleAxis.arm_axis[2].position = q3;
//    singleAxis.arm_axis[3].position = q4;

//    if (_t == 0.0)
//        _t = 0.0;
//    else
//        _t -= LOOPRATE;

//    jntTwoTemp = 1*jntTwo.a0 + 1*jntTwo.a1*_t + 1*jntTwo.a2*pow(_t,2) +
//           1*jntTwo.a3*pow(_t,3) + 1*jntTwo.a4*pow(_t,4) + 1*jntTwo.a5*pow(_t,5);
//    pitchTemp = 1*orientCo.a0 + 1*orientCo.a1*_t + 1*orientCo.a2*pow(_t,2) +
//           1*orientCo.a3*pow(_t,3) + 1*orientCo.a4*pow(_t,4) + 1*orientCo.a5*pow(_t,5);

//    temp1 = _x/cos(q1) - a1 - sin(jntTwoTemp)*a2 - cos(pitchTemp)*d5;
//    temp2 = _z - d1 - cos(jntTwoTemp)*a2 - sin(pitchTemp)*d5;
//    q3Minus = atan2(temp1, temp2) - jntTwoTemp;
//    q4Minus = M_PI/2 - pitchTemp - jntTwoTemp - q3;

//    q3Dot = (q3 - q3Minus)/LOOPRATE;
//    q4Dot = -(q4 - q4Minus)/LOOPRATE;

//    singleAxis.arm_axis[2].position = q3;
//    singleAxis.arm_axis[2].velocity = q3Dot;
//    singleAxis.arm_axis[3].position = q4;
//    singleAxis.arm_axis[3].velocity = q4Dot;

//    singleAxis.arm_axis[4].velocity = 0.0;
//    singleAxis.arm_axis[4].position = 0.0;

    ROS_INFO("q2.velocity = %.f", singleAxis.arm_axis[1].velocity);
    ROS_INFO("q3.velocity = %.f", singleAxis.arm_axis[2].velocity);
    ROS_INFO("q4.velocity = %.f", singleAxis.arm_axis[3].velocity);
}

void motionPlanning::moveToPose() {
    if(calculation_done) {
        timer.start();
        busy = true;
    }
    else {
        ROS_INFO("Calculation not ready!");
        exit(1);
    }
}

void motionPlanning::armIsBusy() {
    while (busy)
        ros::Duration(0.5).sleep();
}

void motionPlanning::moveGripper(double dist) {
    youBotArm->setGripper(dist);
    moveGripper();
}

void motionPlanning::moveGripper() {
    youBotArm->armPositionCommand.positions = youBotArm->gripperJointPositions;
    youBotArm->gripperPositionPublisher.publish(youBotArm->armPositionCommand);
}



void motionPlanning::checkJointLimit() {
    /*
    for (size_t k = 0; k < ARMJOINTS; k++) {
        if(armJointVelocities.velocities[k].value > 0.05)
            positiv = true;
        else
            positiv = false;
        if(armJointVelocities.velocities[k].value < -0.05)
            negativ = true;
        else
            negativ = false;

        if((armJointPositions.positions[k].value > (q_max[k] - 0.02)) && positiv) {
            if (ramp_started) {
                gettimeofday(&detail_ramp,NULL);
                ramp_start_time = double(detail_ramp.tv_sec + 0.000001*detail_ramp.tv_usec);
                ramp_start_time = ramp_start_time - start_time;
                ramp_started = false;
            }
            armJointVelocities.velocities[k].value =  -pow(armJointVelocities.velocities[k].value,2)/(2*0.015)*(t - ramp_start_time)
                                                      + armJointVelocities.velocities[k].value;
            ROS_INFO("Maximalanschlag_%d erreicht!", (int)k+1);
            if(armJointVelocities.velocities[k].value < 0.1) {
                armJointVelocities.velocities[k].value = 0.0;
            }
        }

        if((armJointPositions.positions[k].value < (q_min[k] + 0.02)) && negativ) {
            if (ramp_started) {
                gettimeofday(&detail_ramp,NULL);
                ramp_start_time = double(detail_ramp.tv_sec + 0.000001*detail_ramp.tv_usec);
                ramp_start_time = ramp_start_time - start_time;
                ramp_started = false;
            }
            armJointVelocities.velocities[k].value =  pow(armJointVelocities.velocities[k].value,2)/(2*0.015)*(t - ramp_start_time)
                                                      - armJointVelocities.velocities[k].value;
            ROS_INFO("Minimalanschlag_%d erreicht!", (int)k+1);
            if(armJointVelocities.velocities[k].value > 0.1) {
                armJointVelocities.velocities[k].value = 0.0;
            }
        }
    }
    */
}


void motionPlanning::slerp(tf::Quaternion _q0, tf::Quaternion _q1, double _tRel, tf::Quaternion &_q) {

    tf::Quaternion quatE;
    //tf::Quaternion quatEDot;
    tf::Quaternion temp;
    tf::Quaternion expQuat;
    //tf::Quaternion logQuat;
    double theta;
    double uVec[3];


    if(_q0 != _q1) {
        /**
         *  Quaternionen normieren
         */
        _q0.normalized();
        _q1.normalized();

        /**
         *  Den kürzesten Weg bestimmen
         */
        if(_q0.dot(_q1) < 0)
            _q1.operator *=(-1);

        /**
         *  temp = q0^(-1) * q1
         */
        temp = _q0.inverse();
        temp.operator *=(_q1);

        /**
         *  temp = (w, V3) -> Eulerdarstellung: q = cos(theta) + U3*sin(theta)
         *  mit     theta = arccos(temp.w)
         *          U3     = V3 / sin(theta)
         */
        theta = acos(temp.w());
        uVec[0] = temp.x()/sin(theta);
        uVec[1] = temp.y()/sin(theta);
        uVec[2] = temp.z()/sin(theta);

        /**
         *  Exponentieren: q^t = cos(theta*t) + U3*sin(theta*t)
         */
        expQuat.setW(cos(theta * _tRel));
        expQuat.setX(uVec[0] * sin(theta*_tRel));
        expQuat.setY(uVec[1] * sin(theta*_tRel));
        expQuat.setZ(uVec[2] * sin(theta*_tRel));

        /**
         *  SLERP = q0(q0^(-1) * q1)^t
         */
        quatE = _q0;
        quatE.operator *=(expQuat);

        /**
         *  Logarithmieren: q^t = 0 + U3*theta
         */
        //logQuat.setW(0.0);
        //logQuat.setX(uVec[0]*theta);
        //logQuat.setY(uVec[1]*theta);
        //logQuat.setZ(uVec[2]*theta);

        /**
         *  SLERP' = SLERP * log(q0^(-1))
         */
        //quatEDot = quatE;
        //quatEDot.operator *=(logQuat);

        _q = quatE;
    }
    else
        _q = _q0;
}

void motionPlanning::getOmega(tf::Quaternion _quatStart, tf::Quaternion _quatEnd, double _t, Eigen::Vector3d &_omega) {
    double tempSum1;
    double tempSum2;
    double angularVelocity;
    double axisAngle[4];
    Vector3d rotationVecRates;
    double rotationVec[3];
    double rotVecLength;
    double a;
    double trel;
    double trelMinus;
    double trelPlus;
    tf::Quaternion quat;
    tf::Quaternion quatTempMinus;
    tf::Quaternion quatTempPlus;
    MatrixXd WMat(3,4);
    MatrixXd GMat(4,3);
    Matrix3d VMat;

    if (_t == 0) {
        quatRot = _quatStart.inverse();
        quatRot.operator *=(_quatEnd);
    }

    trel = timeRel.a3*pow(_t,3) + timeRel.a4*pow(_t,4) + timeRel.a5*pow(_t,5);
    trelMinus = timeRel.a3*pow(_t-LOOPRATE,3) + timeRel.a4*pow(_t-LOOPRATE,4) + timeRel.a5*pow(_t-LOOPRATE,5);
    trelPlus = timeRel.a3*pow(_t+LOOPRATE,3) + timeRel.a4*pow(_t+LOOPRATE,4) + timeRel.a5*pow(_t+LOOPRATE,5);

    if(trelMinus < 0.0)
        trelMinus = 0.0;

    if(trelPlus > 1.0)
        trelPlus = 1.0;

    slerp( _quatStart, _quatEnd, trel, quat);
    slerp( _quatStart, _quatEnd, trelMinus, quatTempMinus);
    slerp( _quatStart, _quatEnd, trelPlus, quatTempPlus);


   tf::Matrix3x3(quat).getEulerZYX(degreeOfFreedom[3].position, degreeOfFreedom[4].position, degreeOfFreedom[5].position);

    tempSum1 = (quat.operator -(quatTempMinus)).length();
    tempSum2 = (quat.operator -(quatTempPlus)).length();
    angularVelocity = 100*(tempSum1 + tempSum2)/2.0;

    if(angularVelocity != 0.) {
        axisAngle[0] = 2.0*acos(quatRot.w());
        axisAngle[1] = quatRot.x()/sqrt(1 - pow(quatRot.w(),2));
        axisAngle[2] = quatRot.y()/sqrt(1 - pow(quatRot.w(),2));
        axisAngle[3] = quatRot.z()/sqrt(1 - pow(quatRot.w(),2));

        rotationVec[0] = axisAngle[0] * axisAngle[1];
        rotationVec[1] = axisAngle[0] * axisAngle[2];
        rotationVec[2] = axisAngle[0] * axisAngle[3];
        rotVecLength = sqrt(pow(rotationVec[0],2) + pow(rotationVec[1],2) + pow(rotationVec[2],2));

        rotationVecRates(0) = angularVelocity * axisAngle[1];
        rotationVecRates(1) = angularVelocity * axisAngle[2];
        rotationVecRates(2) = angularVelocity * axisAngle[3];

        a = cos(rotVecLength/2.0)*rotVecLength - 2.0*sin(rotVecLength/2.0);

        WMat<< rotationVec[0]*sin(rotVecLength/2.0), rotVecLength*cos(rotVecLength/2.0), -rotationVec[2]*sin(rotVecLength/2.0), rotationVec[1]*sin(rotVecLength/2.0),
                -rotationVec[1]*sin(rotVecLength/2.0), rotationVec[2]*sin(rotVecLength/2.0), rotVecLength*cos(rotVecLength/2.0), -rotationVec[0]*sin(rotVecLength/2.0),
                -rotationVec[2]*sin(rotVecLength/2.0), -rotationVec[1]*sin(rotVecLength/2.0), rotationVec[0]*sin(rotVecLength/2.0), rotVecLength*cos(rotVecLength/2.0);
        WMat = 1.0/rotVecLength*WMat;

        GMat << -rotationVec[0]*rotVecLength*sin(rotVecLength/2.0), -rotationVec[1]*rotVecLength*sin(rotVecLength/2.0), -rotationVec[2]*rotVecLength*sin(rotVecLength/2.0),
                2.0*rotVecLength*sin(rotVecLength/2.0) + pow(rotationVec[0],2)*a , rotationVec[0]*rotationVec[1]*a, rotationVec[0]*rotationVec[2]*a,
                rotationVec[0]*rotationVec[1]*a, 2.0*rotVecLength*sin(rotVecLength/2.0) + pow(rotationVec[1],2)*a, rotationVec[1]*rotationVec[2]*a,
                rotationVec[0]*rotationVec[2]*a, rotationVec[1]*rotationVec[2]*a, 2.0*rotVecLength*sin(rotVecLength/2.0) + pow(rotationVec[2],2)*a;

        VMat = WMat * GMat;

        _omega = 2 * VMat * rotationVecRates;
    }
    else {
        _omega(0) = 0.;
        _omega(1) = 0.;
        _omega(2) = 0.;
    }
}

void motionPlanning::moveRobot(){
    timerBase.start();
    busy = true;
}

void motionPlanning::setTargetBasePosition(double _x, double _y, double _phi) {

    baseX.position = _x;        baseX.velocity = 0.2;       baseX.acceleration = 0.2;
    baseY.position = _y;        baseY.velocity = 0.2;       baseY.acceleration = 0.2;
    basePhi.position = _phi;    basePhi.velocity = 0.5;     basePhi.acceleration = 0.5;
    if(_x == 0 && _y == 0 )
        onlyOrientation = true;
}

void motionPlanning::BaseProperties() {
    double t_vmax = 0.0;
    double t_amax = 0.0;
    double t_max = 0.0;
    double t_f = 0.0;
    COEFFS tempCoeffs;
    vector <PARAM> baseDiff;
    vector <PARAM> init, goal;
    PARAM temp;

    baseDiff.push_back(baseX);
    baseDiff.push_back(baseY);
    baseDiff.push_back(basePhi);

    for(size_t k = 0; k < 3; k++) {
        temp.position = 0.0;
        temp.velocity = 0.0;
        temp.acceleration = 0.0;
        init.push_back(temp);
    }

    for(size_t k = 0; k < 3; k++) {
        temp.position = baseDiff[k].position;
        temp.velocity = 0.0;
        temp.acceleration = 0.0;
        goal.push_back(temp);
    }


    //Berechnen der Maximalen Trajektorierenzeit auf Basis der Maximalen Geschwindikeiten/Beschleunigungen
    for (size_t k = 0; k < 2; k++) {
        t_vmax = 15.0/8.0*fabs(baseDiff[k].position)/baseDiff[k].velocity;
        t_amax = sqrt(10.0/sqrt(3.0)*fabs(baseDiff[k].position)/baseDiff[k].acceleration);
        t_max = max(t_vmax, t_amax);
        if (t_max > t_f)
            t_f = t_max;
    }
    t_f_base[0] = t_f;
    t_f = 0.0;

    //Berechnen der Maximalen Trajektorierenzeit auf Basis der Maximalen Geschwindikeiten/Beschleunigungen
    t_vmax = 15.0/8.0*fabs(baseDiff[2].position)/baseDiff[2].velocity;
    t_amax = sqrt(10.0/sqrt(3.0)*fabs(baseDiff[2].position)/baseDiff[2].acceleration);
    t_max = max(t_vmax, t_amax);
    if (t_max > t_f)
        t_f = t_max;
    t_f_base[1] = t_f;
    t_f = 0.0;

    //Bestimmung der Koeffizienten für die Position
    for (size_t k = 0; k < 2; k++) {
        setPolynomCoefficients(init[k], goal[k], t_f_base[0], tempCoeffs);
        baseCoeffs.push_back(tempCoeffs);
    }

    //Bestimmung der Koeffizienten für die Orientierung
        setPolynomCoefficients(init[2], goal[2], t_f_base[1], tempCoeffs);
        baseCoeffs.push_back(tempCoeffs);

}

void motionPlanning::offlineBasePlanning() {
    vector <PARAM> degreeOfFreedom;
    double wrappedTime = 0.0;
    degreeOfFreedom.clear();
    degreeOfFreedom.resize(3);
    double angle = 0.0;
    double X[3] = {0.0,0.0,0.0};
    double XPos[3] = {0.0,0.0,0.0};

//    listener.lookupTransform("base_link", "map", ros::Time(0), transform);
//    firstBasePose.position.x = transform.getOrigin().getX();
//    firstBasePose.position.y = transform.getOrigin().getY();
    this->startBasePose = this->lastBasePose;
    ROS_INFO("startBasePose = %f", startBasePose.orientation.w);

    BaseProperties();

    wrappedTime = 0.0;

    while (wrappedTime < t_f_base[0])  {
        for (size_t k = 0; k < 2; k++) {
            setPolynomFunction(baseCoeffs[k], wrappedTime, degreeOfFreedom[k]);
//            X[k] = degreeOfFreedom[k].velocity;
//            XPos[k] = degreeOfFreedom[k].position;
            singleWheel.arm_axis[k].position = degreeOfFreedom[k].position;
            singleWheel.arm_axis[k].velocity = degreeOfFreedom[k].velocity;

        }
//        angle = degreeOfFreedom[2].position;
//        singleWheel.arm_axis[0].velocity = X[0];
//        singleWheel.arm_axis[1].velocity = X[1];
//        singleWheel.arm_axis[2].velocity = X[2];
//        singleWheel.arm_axis[0].velocity = X[0]*cos(angle) + X[1]*sin(angle);
//        singleWheel.arm_axis[1].velocity = -X[0]*sin(angle) + X[1]*cos(angle);
//        singleWheel.arm_axis[2].velocity = X[2];
//        singleWheel.arm_axis[0].position = XPos[0];//*cos(XPos[2]) + XPos[1]*sin(XPos[2]);
//        singleWheel.arm_axis[1].position = XPos[1];//*sin(XPos[2]) + XPos[1]*cos(XPos[2]);
//        singleWheel.arm_axis[2].position = XPos[2];
        wrappedTime = wrappedTime + LOOPRATE;
        timeIncBaseTrans.push_back(singleWheel);
    }
    singleWheel.arm_axis.resize(1);
    wrappedTime = 0.0;
    while (wrappedTime < t_f_base[1])  {
        setPolynomFunction(baseCoeffs[2], wrappedTime, degreeOfFreedom[2]);
        singleWheel.arm_axis[0].position = degreeOfFreedom[2].position;
        singleWheel.arm_axis[0].velocity = degreeOfFreedom[2].velocity;

        wrappedTime = wrappedTime + LOOPRATE;
        timeIncBaseRot.push_back(singleWheel);

    }
}

void motionPlanning::test_run(double _x, double _y, double _yaw) {
    geometry_msgs::PoseStamped inMap;
    geometry_msgs::PoseStamped outBase;
//    listener.lookupTransform("\baselink", ros::Time(0), "\map", transform);
////    listener.lookupTransform("\baselink", ros::Time(0), );

////    listener.lookupTransform("\baselink",  ros::Time(0), "\map",transform);
//    //tf::Quaternion qStart(_curr_pose.orientation.x, _curr_pose.orientation.y, _curr_pose.orientation.z, _curr_pose.orientation.w);
//    btQuaternion workQuat = transform.getRotation();
//    btScalar x, y, z, roll, pitch, yaw;
//    btMatrix3x3(workQuat).getRPY(roll, pitch, yaw);
    transform.setOrigin(tf::Vector3(_x,_y, 0.0));
    transform.setRotation(tf::Quaternion(_yaw, 0, 0));
    tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion rotation = transform.getRotation();
    inMap.pose.position.x = origin.getX();
    inMap.pose.position.y = origin.getY();
    inMap.pose.position.z = 0;

    inMap.pose.orientation.w = rotation.getW();
    inMap.pose.orientation.x = rotation.getX();
    inMap.pose.orientation.y = rotation.getY();
    inMap.pose.orientation.z = rotation.getZ();

    listener.transformPose("base_link", ros::Time(0), inMap, "map", outBase);
    listener.lookupTransform("base_link", "map", ros::Time(0), transform);

//    firstBasePose.position.x = transform.getOrigin().getX();

//    outBase.pose.orientation.


//    tf::Quaternion _q1(_curr_pose.pose.orientation.x, _curr_pose.orientation.y, _curr_pose.orientation.z, _curr_pose.orientation.w);
    tf::Quaternion _q2(outBase.pose.orientation.x, outBase.pose.orientation.y, outBase.pose.orientation.z, outBase.pose.orientation.w);

    tf::Quaternion _q3 = transform.inverse() * _q2;

    double yaw = fabs(tf::getYaw(_q3));

    double x = (double)transform.getOrigin().getX() - outBase.pose.position.x;
    double y = (double)transform.getOrigin().getY() - outBase.pose.position.y;



    setTargetBasePosition( x, y, yaw);
//}


}






