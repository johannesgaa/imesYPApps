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
 * @file   Reglerauslegung.cpp
 * @author eduard (eduardpopp@web.de)
 * @date   12.11.2012
 *
 * @brief  Filedescription
 */
#include <../test/include/controlUnit.hpp>

controlUnit::controlUnit() {
    //Laden der Configfile aus dem Paramterraum
    string path = ros::package::getPath("youbot_manipulator");
    stringstream temp_cf;
    temp_cf << path << "/config/youbot-manipulator.cfg";
    string configFile = temp_cf.str();
    youBotArm = new youbot_arm_kinematics(configFile, "arm_link_0");
    //n.param<string>("../motion_planning/configFile", this->configFile, temp_cf.str());

    //Initialisierung der Vektoren
    armJointVelocities.velocities.resize(ARMJOINTS);
    armJointPositions.positions.resize(ARMJOINTS);
    controler.positions.resize(4);

    stringstream jointName;
    timeIt = 0.00;
    e = 0.0000;
    eSum = 0.0000;
    eAlt = 0.0000;
    F = 0.0;


    for (unsigned int idx = 0; idx < ARMJOINTS; ++idx) {
        jointName.str("");
        jointName << "arm_joint_" << (idx + 1);

        armJointPositions.positions[idx].joint_uri = jointName.str();
        armJointPositions.positions[idx].unit = boost::units::to_string(boost::units::si::radians);
        armJointPositions.positions[idx].value = 0.0;

        armJointVelocities.velocities[idx].joint_uri = jointName.str();
        armJointVelocities.velocities[idx].unit = boost::units::to_string(boost::units::si::radians_per_second);
        armJointVelocities.velocities[idx].value = 0.0;

    }
    //Publisher und Subscriber Intialisieren
    timer = n.createTimer(ros::Duration(LOOPRATE), &controlUnit::TimerCallback, this, false, false);
    jointPosVelSubscriber = n.subscribe("/arm_1/joint_states", 1, &controlUnit::JointPositionCallback, this);
    jointVelocityPublisher = n.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);
    controlerPublisher = n.advertise<brics_actuator::JointPositions>("/regler",1);

    while(ros::ok() && jointVelocityPublisher.getNumSubscribers()== 0){
        ROS_INFO("Waiting for subscribers on /arm_1/arm_controller/velocity_command");
        ros::Duration(1.0).sleep();
    }
}

controlUnit::~controlUnit() {
}

void controlUnit::JointPositionCallback(const sensor_msgs::JointState &jointPositionCommand) {
    for (size_t m = 0; m < jointPositionCommand.name.size(); m++) {
        for (size_t n = 0; n < armJointPositions.positions.size(); n++) {
            if (armJointPositions.positions[n].joint_uri == jointPositionCommand.name[m]) {
                armJointPositions.positions[n].value = jointPositionCommand.position[m];
                break;
            }
        }
    }
}

void controlUnit::TimerCallback(const ros::TimerEvent &ex) {
    //---------------------------------------------Regler-Auslegung -------------------------------------------------//

    int plot_e = 1;
    int plot_esum = 2;
    int plot_pos = 0;

    double KP = 0.12;
    double KI = 0.02;
    double KD = 0.00;
    double B = 2;
    double H = 20;


    e = 165.0 - armJointPositions.positions[JOINT5].value * 180.0/M_PI;
    eSum = e + eSum;
    //Anti Windup
    // + KD*(e - eAlt)/LOOPRATE
    F = KP*e + KI*LOOPRATE*eSum;
    armJointVelocities.velocities[JOINT5].value = F;
    if (eSum > H)
        eSum = eSum - B*(eSum - H);
    if (eSum < (-1)*H)
        eSum = eSum - B*(eSum + H);
//        if (eSum > H)
//            eSum = H;
//        if (eSum < (-1)*H)
//            eSum = -H;

    controler.positions[plot_e].value = e;
    controler.positions[plot_esum].value = eSum;
    controler.positions[plot_pos].value = armJointPositions.positions[JOINT5].value * 180/M_PI;
    controler.positions[3].value = F;

    //Messwerte aufzeichnen
    string path = "/home/eduard/ros_stacks/imes_youbot_pkg/youbot_motion_control/test/logfiles/JointFuenf/controller_";
    stringstream ss;
    ss << path << KP << "_" << (int)KI+1 << "_" << H << "_" << (int)B  << ".txt";
    string file;
    ss >> file;
    char * buffer = new char[file.length()];
    strcpy(buffer,file.c_str());
    fstream f;
    f.open(buffer, ios::app | ios::out);
    f << timeIt*LOOPRATE << "\t  " << (armJointPositions.positions[JOINT5].value*180/M_PI - 160.) << "\t  " << e << "\t  " << eSum <<  endl;
    f.close();

    controlerPublisher.publish(controler);
    jointVelocityPublisher.publish(armJointVelocities);
    timeIt++;

//    ROS_INFO("Zeit: %f", (double)(timeIt*LOOPRATE));
    eAlt = e;
}

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controlUnit");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Time begin, end;
    char keyboard_input;

    controlUnit cUnit;

    while (ros::ok())
    {

        keyboard_input = (char)kbhit();
        if(keyboard_input == 'g') {

            cUnit.youBotArm->setJntPos(0.0 * M_PI/180, 0.0 * M_PI/180, 0. * M_PI/180,
                                       0.0 * M_PI/180, 155. * M_PI/180);
            cUnit.youBotArm->moveToPose();
            ros::Duration(1.5).sleep();
            cUnit.youBotArm->setJntPos(0.0 * M_PI/180, 0.0 * M_PI/180, 0. * M_PI/180,
                                       0.0 * M_PI/180, 161.78 * M_PI/180);
            cUnit.youBotArm->moveToPose();
            ros::Duration(1.5).sleep();
            begin = ros::Time::now();
            cUnit.timer.start();

        }
        end = ros::Time::now();
        if(end.toSec() > (begin.toSec() + 5.0)) {
            cUnit.timer.stop();
            cUnit.timeIt = 0;

        }


    }

    return 0;
}
