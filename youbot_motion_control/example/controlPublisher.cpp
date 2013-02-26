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
 * @file   controlPublisher.cpp
 * @author Eduard Popp (eduardpopp@web.de)
 * @date   26. September 2012
 *
 * @brief  Filedescription
 */

#include <youbot_motion_control/motionPlanning.hpp>
#include <kdl/kdl.hpp>
#include <../example/testApplication.cpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mPlanning");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    motionPlanning *motionPlanner;
    motionPlanner = new motionPlanning();


    while (ros::ok())
    {
        char keyboard_input = (char)kbhit();
        //ROS_INFO("test");
        if((keyboard_input == 'g') || (motionPlanner->busy)) {
//            motionPlanner->newArmMove();
//            motionPlanner->setManipulatorVelocity(0.1);
//            motionPlanner->moveArm( 0.05, 0.0, 0.0, 00.0 * M_PI/180, 0.0 * M_PI/180);
//            motionPlanner->offlineMotionPlanning();
//            ROS_INFO("Berechnung ausgeführt");
//            motionPlanner->moveToPose();
//            ROS_INFO("Arm is busy");
//            motionPlanner->armIsBusy();
            /*
            mPlanner.newTrajectory();
            mPlanner.setPathPoint(100.5 * M_PI/180, 37.5 * M_PI/180, -43.5 * M_PI/180,
                                      97.5 * M_PI/180, 86.0 * M_PI/180, 0.06);
            mPlanner.setTargetJointVelocity(1.0);
            mPlanner.setPathPoint(169.0 * M_PI/180, 75.6 * M_PI/180, -87.5 * M_PI/180,
                                      195 * M_PI/180, 172.0 * M_PI/180);
            */

            //ROS_INFO("Arm is not busy!");
            //unload_bottle(motionPlanner);
            //unload_glass(motionPlanner);
            //serve_drink(motionPlanner);
            //motionPlanner->test_run();

//            motionPlanner->newTrajectory();
//            motionPlanner->setPathPoint(1.0 * M_PI/180, 20.5 * M_PI/180, -1.0 * M_PI/180,
//                                      10.0 * M_PI/180, 7.0 * M_PI/180);
//            motionPlanner->setTargetJointVelocity(1.0);
//            motionPlanner->setPathPoint(169.0 * M_PI/180, 75.6 * M_PI/180, -87.5 * M_PI/180,
//                                      120.5 * M_PI/180, 172.0 * M_PI/180);
//            motionPlanner->setPathPoint(1.0 * M_PI/180, 0.6 * M_PI/180, -1.5 * M_PI/180,
//                                      20.0* M_PI/180, 7.0 * M_PI/180);
//            ROS_INFO("Hallo");
//            motionPlanner->newTrajectory();
////            motionPlanner->setTargetJointVelocity(1.0);
////            motionPlanner->setPathPoint(1.0 * M_PI/180, 1.0 * M_PI/180, -1.0 * M_PI/180,
////                                      6.0 * M_PI/180, 7.0 * M_PI/180);
////            motionPlanner->setTargetJointVelocity(0.7);
//            motionPlanner->setPathPoint(140.0 * M_PI/180, 1.0 * M_PI/180, -1.0 * M_PI/180,
//                                      6.0 * M_PI/180, 7.0 * M_PI/180);
//            motionPlanner->setPathPoint(20.0 * M_PI/180, 1.0 * M_PI/180, -1.0 * M_PI/180,
//                                      6.0* M_PI/180, 7.0 * M_PI/180);
////            motionPlanner->setPathPoint(230.0 * M_PI/180, 74.0 * M_PI/180, -115.0 * M_PI/180,
////                                      139.0* M_PI/180, 172.0 * M_PI/180);

//            motionPlanner->offlineMotionPlanning();
//            motionPlanner->moveToPose();
//            motionPlanner->armIsBusy();
//            ROS_INFO("cuuu");
////            motionPlanner->newArmMove();
//            motionPlanner->setTargetBasePosition(0.0, 0.0, 90.0*M_PI/180);
//            motionPlanner->offlineBasePlanning();
//            motionPlanner->moveRobot();
//            motionPlanner->armIsBusy();

            motionPlanner->newArmMove();
//            motionPlanner->test_run(2,0,20);
            motionPlanner->setTargetBasePosition(0.0, 0.0, 90.0*M_PI/180);
            motionPlanner->offlineBasePlanning();
            motionPlanner->moveRobot();
            motionPlanner->armIsBusy();


        }

    }
    return 0;
}
