/************************************************************************
 * @brief
 *
 *
 *
 * @file       youbot_launcher_server.h
 * @author     Andreas Fregin (A.Fregin@live.de)
 * @date       2013-02-23
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <ros/ros.h>
#include <youbot_showroom/launcher_message.h>

/**
 * @brief laucher class
 */
class launcher_server {

public:

	/**
	 * @brief constructor
	 */
	launcher_server();

	/**
	 * @brief destructor
	 */
	~launcher_server();

private:

	/**
	 * @brief callback method
	 * @param msg topic message
	 */
	void launcher_callback(const youbot_showroom::launcher_message& msg);


	ros::NodeHandle nh;					///< Node handle
	ros::Subscriber sub;				///< ROS Subscriber

	bool DemoRunning;					///< inidicates if a Demo is currently running
	std::string DemoName;				///< name of the Demo that is running - empty if no Demo is running
	std::vector<std::string> Demo;					///< array of demo launch shell commands
};
