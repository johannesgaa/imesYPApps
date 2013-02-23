/************************************************************************
 * @brief
 *
 *
 *
 * @file       youbot_launcher_server.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2013-02-23
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <youbot_showroom/youbot_launcher_server.h>

launcher_server::launcher_server() {

	ROS_INFO("IMES Demo Launcher Server initializing..");

	DemoRunning = false;
	nh.setParam("/launcher_server/DemoRunning", DemoRunning);
	nh.setParam("/launcher_server/DemoName", "");

	// IMPORTANT: Add "&" at the end of each command !!!
	Demo.clear();
	Demo.push_back("roslaunch youbot_showroom/launch/embedded_world.launch &");
//	Demo[1] = "roslaunch ";
//	Demo[2] = "roslaunch ";
//	Demo[3] = "roslaunch ";
//	Demo[4] = "roslaunch ";
//	Demo[5] = "roslaunch ";
//	Demo[6] = "roslaunch ";
//	Demo[7] = "roslaunch ";
	ROS_INFO(".. loaded %i demo command(s) ..", (int)Demo.size());

	sub = nh.subscribe("launch_demo", 1000, &launcher_server::launcher_callback, this);

	ROS_INFO(".. done");

	if (sub.getNumPublishers() < 1) {
		printf("/n");
		ROS_INFO("Waiting for Publishers ..");
	}
}

launcher_server::~launcher_server() {};

void launcher_server::launcher_callback(const youbot_showroom::launcher_message& msg) {

	// NAME = "" && DURATION ==  0   ----> actual demo ended
	if (msg.demoName == "" && msg.duration.toSec() == 0.) {
		ROS_INFO("Actual demo terminated");
		DemoRunning = false;
		nh.setParam("/launcher_server/DemoRunning", DemoRunning);
		nh.setParam("/launcher_server/DemoName", "");

	// New Demo
	} else {

		// Another demo running:
		if (DemoRunning == true) {
			ROS_WARN("Request to run %s cannot be executed - another demo is currently running", msg.demoName.c_str());

		//	No demo running
		} else{
			bool err = system(Demo[msg.demoNumber-1].c_str());
			if (err == false) {
				DemoRunning = true;
				nh.setParam("/launcher_server/DemoRunning", DemoRunning);
				nh.setParam("/launcher_server/DemoName", msg.demoName);
				ROS_INFO("successfully launched demo %d", msg.demoNumber);
			} else {
				ROS_ERROR("Something went wrong - system retured not 0");
			}
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_launcher_server");

	launcher_server my_server;


	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(1).sleep();
	}
	return 0;
}
