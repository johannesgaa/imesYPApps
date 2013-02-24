/************************************************************************
 * @brief      youbot demonstration-project
 *
 * 			   - This is a part of youbot_showroom
 *				 The project is doing object detection usind SIFT feature
 *				 detection (imes_vision/feature_detector) using a moving
 *				 webcam mounted on a youbot manipulator.
 *
 *			   - This was programmed for Embedded worlds exposition at
 *			     Nuernberg 2013
 *
 *
 * @file       embedded_wolrds_2013.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2013-02-21
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/


#include <youbot_showroom/embedded_worlds_2013.h>

/**
 * @brief main function
 */
int main(int argc, char **argv) {
	ROS_INFO("YouBot@embedded_worlds_2013 initializing..");
	ros::init(argc, argv, "embedded_wolrds_2013");

	nh = new ros::NodeHandle;

	pub = nh->advertise<youbot_showroom::launcher_message>("/launcher_server", 1);

	ROS_INFO(".. please start your detector ..");

	ROS_INFO("..loading manipulator class ..");
    // load manipulator class
	std::string _configFile = ros::package::getPath("youbot_driver");
	_configFile.append("/config/youbot-manipulator.cfg");
	manipulator = new youbot_arm_kinematics(_configFile, "arm_link_0");

	double ARM_LEFT_POSITION =		149		*KDL::deg2rad;
	double ARM_MIDDLE_POSITION = 	169		*KDL::deg2rad;
	double ARM_RIGHT_POSITION =		189		*KDL::deg2rad;
	double position[6] = {ARM_MIDDLE_POSITION, 19.5 *KDL::deg2rad, -61.0 *KDL::deg2rad, 195.0 *KDL::deg2rad, 172.0 *KDL::deg2rad, 0.023};

	ROS_INFO(".. done!");
	ROS_INFO("Loop started.");
	// while okay move arm and detect!
	while (ros::ok()) {

		manipulator->setJntPos(ARM_LEFT_POSITION, position[1], position[2], position[3], position[4], position[5]);
		manipulator->moveToPose();
		ros::Duration(5.0).sleep();

		manipulator->setJntPos(ARM_MIDDLE_POSITION, position[1], position[2], position[3], position[4], position[5]);
		manipulator->moveToPose();
		ros::Duration(5.0).sleep();

		manipulator->setJntPos(ARM_RIGHT_POSITION, position[1], position[2], position[3], position[4], position[5]);
		manipulator->moveToPose();
		ros::Duration(5.0).sleep();

		manipulator->setJntPos(ARM_MIDDLE_POSITION, position[1], position[2], position[3], position[4], position[5]);
		manipulator->moveToPose();
		ros::Duration(5.0).sleep();
	}

	youbot_showroom::launcher_message msg;
	msg.demoNumber = 0;
	msg.demoName = "";
	msg.duration.sec = 0;
	msg.duration.nsec = 0;
	pub.publish(msg);

	return 0;
}
