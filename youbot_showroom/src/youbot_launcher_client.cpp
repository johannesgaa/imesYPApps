/************************************************************************
 * @brief     
 *
 *
 * @file       youbot_launcher_client.cpp
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       23.02.2013
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <ros/ros.h>
#include <youbot_showroom/launcher_message.h>

class youbot_launcher_client {

public:
	youbot_launcher_client();

	~youbot_launcher_client();

	ros::NodeHandle nh;
	ros::Publisher pub;

	void chooseDemo();
};

youbot_launcher_client::youbot_launcher_client() {

	ROS_INFO("Test client initializing..");
	pub = nh.advertise<youbot_showroom::launcher_message>("/launch_demo", 1);

	ROS_INFO(".. done.");
}

youbot_launcher_client::~youbot_launcher_client() {};

void youbot_launcher_client::chooseDemo() {
	while (ros::ok()) {
		printf("\n\n");
		printf("Please type in a number between 1 and 8: ");
		int demo;
		std::cin >> demo;;
		if ((demo >0) && (demo < 9)) {
			youbot_showroom::launcher_message msg;
			msg.demoName = demo;
			msg.demoNumber = demo;
			msg.duration.sec = 2;
			msg.duration.nsec = 2;
			pub.publish(msg);
			ROS_INFO("sent");
		}
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_client");

	youbot_launcher_client my_client;
	my_client.chooseDemo();

	return 0;
}
