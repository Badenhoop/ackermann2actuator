//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_COMMON_H
#define ACKERMANN2ACTUATOR_COMMON_H

#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <thread>

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
	// Override SIGINT handler
	ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
	signal(SIGINT, sigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	MeasuringProcess process{};
	std::thread experimentThread{[&]{ process.run(); }};

	while (process.isRunning() && !requestShutdownFlag)
		ros::spinOnce();

	process.cancel();
	experimentThread.join();

	ros::shutdown();
}

}

#endif //ACKERMANN2ACTUATOR_COMMON_H
