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
 * @file   meinTest.cpp
 * @author eduard (eduardpopp@web.de)
 * @date   12.11.2012
 *
 * @brief  Filedescription
 */


int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF && ch > 0)
        return ch;

    return 0;
}

void unload_bottle(motionPlanning *mPlanner) {
    ROS_INFO("start unload bottle");

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(202.0 * KDL::deg2rad, 28.0 * KDL::deg2rad, -65.0 * KDL::deg2rad, 185.0 * KDL::deg2rad, 172.0 * KDL::deg2rad, 0.04);

    //mPlanner->moveToPose();
    //mPlanner->armIsBusy();

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(100.0 * KDL::deg2rad, 28.0 * KDL::deg2rad, -65.0 * KDL::deg2rad, 185.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(28.5 * KDL::deg2rad, 50.0 * KDL::deg2rad, -1.0 * KDL::deg2rad, 102.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(28.5 * KDL::deg2rad, 81 * KDL::deg2rad, -1.0 * KDL::deg2rad, 55.85 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(1.0);
    mPlanner->setPathPoint(28.5 * KDL::deg2rad, 112.784 * KDL::deg2rad, -1.0 * KDL::deg2rad, 9.743 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveGripper(0.000);

    mPlanner->setTargetJointVelocity(1.0);
    //mPlanner->setPathPoint(27.5 * KDL::deg2rad, 112.0 * KDL::deg2rad, -9.0 * KDL::deg2rad, 13.5 * KDL::deg2rad, 172.0 * KDL::deg2rad, true);
    mPlanner->setPathPoint(28.5 * KDL::deg2rad, 112.0 * KDL::deg2rad, -4.5 * KDL::deg2rad, 13.5 * KDL::deg2rad, 172.0 * KDL::deg2rad, 0.003);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    ros::Duration(0.5).sleep();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.05);
    mPlanner->setPathPoint(28.5 * KDL::deg2rad, 100.0 * KDL::deg2rad, -4.5 * KDL::deg2rad, 13.5 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(1.0);
    mPlanner->setPathPoint(90.0 * KDL::deg2rad, 100.0 * KDL::deg2rad, -4.5 * KDL::deg2rad, 1.5 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    // hier sicherheitspose
    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(168.5 * KDL::deg2rad, 100.0 * KDL::deg2rad, -4.5 * KDL::deg2rad, 1.5 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    mPlanner->newTrajectory();
    // set bottle
    //mPlanner->setTargetJointVelocity(0.2);
    //mPlanner->setPathPoint(168.5 * KDL::deg2rad, 133.9 * KDL::deg2rad, -26.610 * KDL::deg2rad, 9.977 * KDL::deg2rad, 172.5 * KDL::deg2rad);
    //mPlanner->moveToPose();

    //mPlanner->setTargetJointVelocity(0.05);
    //mPlanner->setPathPoint(169.0 * KDL::deg2rad, 140.0 * KDL::deg2rad, -26.610 * KDL::deg2rad, 6.0 * KDL::deg2rad, 172.5 * KDL::deg2rad);
    //mPlanner->youBotArm->setGripper(0.023);
    //mPlanner->offlineMotionPlanning();
    //mPlanner->moveToPose();
    //mPlanner->armIsBusy();

    //mPlanner->newTrajectory();
    //mPlanner->setTargetJointVelocity(0.1);
    //mPlanner->setPathPoint(169.0 * KDL::deg2rad, 145.0 * KDL::deg2rad, -26.610 * KDL::deg2rad, 2.0 * KDL::deg2rad, 172.5 * KDL::deg2rad);
    mPlanner->setTargetJointVelocity(0.2);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 90.00 * KDL::deg2rad, -27.0 * KDL::deg2rad, 42.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->setTargetJointVelocity(0.2);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 146.0 * KDL::deg2rad, -42.885 * KDL::deg2rad, 13.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveGripper(0.023);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    mPlanner->moveGripper(0.04);

    //ros::Duration(1).sleep();
    //mPlanner->newArmMove();
    //mPlanner->moveArm(-0.005, 0.0, 0.0, 0.0, 0.0, 0.0);
    //mPlanner->offlineMotionPlanning();
    //mPlanner->armIsBusy();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 125.0 * KDL::deg2rad, -6.0 * KDL::deg2rad, 13.0 * KDL::deg2rad, 172.5 * KDL::deg2rad);
    //mPlanner->moveToPose();

    mPlanner->setTargetJointVelocity(1.0);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 100.0 * KDL::deg2rad, -30.0 * KDL::deg2rad, 20.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();
    ROS_INFO("bottle unloaded!");
    return;
}

void unload_glass(motionPlanning *mPlanner) {

    ROS_INFO("start unload glass");

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(1.0);
    mPlanner->setPathPoint(100.0 * KDL::deg2rad, 28.0 * KDL::deg2rad, -65.0 * KDL::deg2rad, 185.0 * KDL::deg2rad, 172.0 * KDL::deg2rad, 0.04);
    //mPlanner->moveGripper(0.023);

    mPlanner->setTargetJointVelocity(1.0);
    mPlanner->setPathPoint(10.0 * KDL::deg2rad, 28.0 * KDL::deg2rad, -65.0 * KDL::deg2rad, 185.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();


    mPlanner->newTrajectory();

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(10.0 * KDL::deg2rad, 50.0 * KDL::deg2rad, -10.0 * KDL::deg2rad, 100.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.1);
    mPlanner->setPathPoint(10.0 * KDL::deg2rad, 81 * KDL::deg2rad, -5.0 * KDL::deg2rad, 55.85 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.08);
    mPlanner->setPathPoint(10.0 * KDL::deg2rad, 112.784 * KDL::deg2rad, -2.0 * KDL::deg2rad, 5.0 * KDL::deg2rad, 172.0 * KDL::deg2rad, 0.003);
    //mPlanner->moveGripper(0.000);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    ros::Duration(0.5).sleep();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.3);
    mPlanner->setPathPoint(10.0 * KDL::deg2rad, 80.0 * KDL::deg2rad, -2.0 * KDL::deg2rad, 20.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(120.0 * KDL::deg2rad, 45.0 * KDL::deg2rad, -100.0 * KDL::deg2rad, 140.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 55.0 * KDL::deg2rad, -98.0 * KDL::deg2rad, 141.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 45.0 * KDL::deg2rad, -50.0 * KDL::deg2rad, 100.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(1.0);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 125.0 * KDL::deg2rad, -43.0 * KDL::deg2rad, 12.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.05);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 144.0 * KDL::deg2rad, -43.0 * KDL::deg2rad, 6.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    //mPlanner->youBotArm->setGripper(0.023);

    //mPlanner->setTargetJointVelocity(0.01);
    //mPlanner->setPathPoint(216.0 * KDL::deg2rad, 132.5 * KDL::deg2rad, -44.0 * KDL::deg2rad, 13.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();
    mPlanner->moveGripper(0.04);

    ros::Duration(0.5).sleep();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.6);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 94.0 * KDL::deg2rad, -10.0 * KDL::deg2rad, 50.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.6);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 76.0 * KDL::deg2rad, -72.0 * KDL::deg2rad, 195.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.6);
    mPlanner->setPathPoint(216.0 * KDL::deg2rad, 35.0 * KDL::deg2rad, -72.0 * KDL::deg2rad, 195.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();mPlanner->moveToPose();
    mPlanner->armIsBusy();

    return;
}

void serve_drink(motionPlanning *mPlanner) {
    mPlanner->moveGripper(0.04);

    ROS_INFO("start serve drink");
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(168.5 * KDL::deg2rad, 100.0 * KDL::deg2rad, -43.0 * KDL::deg2rad, 25.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(168.5 * KDL::deg2rad, 133.9 * KDL::deg2rad, -26.610 * KDL::deg2rad, 9.977 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    //mPlanner->setTargetJointVelocity(0.5);
    //mPlanner->setPathPoint(168.5 * KDL::deg2rad, 133.9 * KDL::deg2rad, -26.610 * KDL::deg2rad, 9.977 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->offlineMotionPlanning();
    //mPlanner->moveToPose();
    //mPlanner->armIsBusy();

    mPlanner->setTargetJointVelocity(0.8);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 136.0 * KDL::deg2rad, -37.885 * KDL::deg2rad, 13.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->youBotArm->setGripper(0.000);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();
    mPlanner->moveGripper(0.003);

    ros::Duration(0.2).sleep();

    //-----------------------Eingießen----------------------------------------------
    // Pose1
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.8);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 90.00 * KDL::deg2rad, -27.0 * KDL::deg2rad, 42.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    // Pose2
    mPlanner->setTargetJointVelocity(0.2);
    mPlanner->setPathPoint(194.0 * KDL::deg2rad, 87.00 * KDL::deg2rad, -16.0 * KDL::deg2rad, 42.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    // Pose3
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.12);
    mPlanner->setPathPoint(195.0 * KDL::deg2rad, 76.8 * KDL::deg2rad, -15.4 * KDL::deg2rad, 53.0 * KDL::deg2rad, 98.4 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    // Pose4
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.05);
    mPlanner->setPathPoint(196.0 * KDL::deg2rad, 70.7 * KDL::deg2rad, -20.0 * KDL::deg2rad, 60.0 * KDL::deg2rad, 75.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    ros::Duration(1.5).sleep();

    // Pose5
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.05);
    mPlanner->setPathPoint(198.0 * KDL::deg2rad, 64.00 * KDL::deg2rad, -31.0 * KDL::deg2rad, 72.0 * KDL::deg2rad, 54.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    ros::Duration(1).sleep();

    // Pose6
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.05);
    mPlanner->setPathPoint(211.0 * KDL::deg2rad, 67.00 * KDL::deg2rad, -65.0 * KDL::deg2rad, 104.7 * KDL::deg2rad, 10.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();

    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    ros::Duration(1.5).sleep();

    // --------------------Zurueckfahren
    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.1);
    mPlanner->setPathPoint(198.0 * KDL::deg2rad, 66.00 * KDL::deg2rad, -31.0 * KDL::deg2rad, 72.0 * KDL::deg2rad, 54.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    mPlanner->setTargetJointVelocity(1);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 90.00 * KDL::deg2rad, -27.0 * KDL::deg2rad, 42.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 129.0 * KDL::deg2rad, -41.5 * KDL::deg2rad, 25.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    mPlanner->newTrajectory();
    mPlanner->setTargetJointVelocity(0.03);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 133.0 * KDL::deg2rad, -41.5 * KDL::deg2rad, 20.0 * KDL::deg2rad, 172.0 * KDL::deg2rad, 0.04);
    //mPlanner->youBotArm->setGripper(0.023);
    //mPlanner->moveToPose();

    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 133.0 * KDL::deg2rad, -37.5 * KDL::deg2rad, 20.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    mPlanner->setTargetJointVelocity(0.5);
    mPlanner->setPathPoint(169.0 * KDL::deg2rad, 100.0 * KDL::deg2rad, -37.0 * KDL::deg2rad, 20.0 * KDL::deg2rad, 172.0 * KDL::deg2rad);
    //mPlanner->moveToPose();

    //mPlanner->setTargetJointVelocity(0.5);
    //mPlanner->setPathPoint(0.0 * KDL::deg2rad, 0.0 * KDL::deg2rad, 0.0 * KDL::deg2rad, 0.0 * KDL::deg2rad, 0.0 * KDL::deg2rad);
    mPlanner->offlineMotionPlanning();
    mPlanner->moveToPose();
    mPlanner->armIsBusy();

    return;
}



