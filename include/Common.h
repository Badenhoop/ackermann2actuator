//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_COMMON_H
#define ACKERMANN2ACTUATOR_COMMON_H

#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <thread>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

// See https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
// for shutdown and SIGINT handling.

namespace a2a
{

extern sig_atomic_t volatile requestShutdownFlag;

void sigIntHandler(int sig);

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

template<typename MeasuringProcess>
void runExperimentNode(int argc, char ** argv, const std::string & nodeName)
{
	using namespace std::chrono_literals;

	// Override SIGINT handler
	ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
	signal(SIGINT, sigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	// set log level
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
		ros::console::notifyLoggerLevelsChanged();

	ros::AsyncSpinner spinner{1};
	spinner.start();

	MeasuringProcess measuringProcess{};

	std::thread signalThread{[&]
	{
		while (requestShutdownFlag)
			std::this_thread::sleep_for(100ms);

		if (measuringProcess.isRunning())
			measuringProcess.cancel();
	}};

	measuringProcess.run();

	// after completion wait for possible pending handlers to finish
	std::this_thread::sleep_for(2s);

	spinner.stop();
	ros::shutdown();

	requestShutdownFlag = 1;
	signalThread.join();
}

float getDistanceFromScan(const sensor_msgs::LaserScan & scan);

}

#endif //ACKERMANN2ACTUATOR_COMMON_H
