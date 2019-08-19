//
// Created by philipp on 15.08.19.
//

#include "../include/Common.h"

namespace a2a
{

extern sig_atomic_t volatile requestShutdownFlag = 0;

void sigIntHandler(int sig)
{
	requestShutdownFlag = 1;
}

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		requestShutdownFlag = 1; // Set flag
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

float getDistanceFromScan(const sensor_msgs::LaserScan & scan)
{
	return *std::min_element(scan.ranges.begin(), scan.ranges.end());
}

}