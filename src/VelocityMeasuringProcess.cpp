//
// Created by philipp on 15.08.19.
//

#include "../include/VelocityMeasuringProcess.h"

namespace a2a
{

void VelocityMeasuringProcess::startMeasuring(double actuatorValue)
{
	ros::param::get(paramNamespace + "/acceleration_distance", measuringDistance);
	ros::param::get(paramNamespace + "/measuring_distance", measuringDistance);

	// keep steering angle neutral
	std_msgs::Float64 msg;
	msg.data = 0.0;
	steeringActuatorPublisher.publish(msg);

	// start driving
	msg.data = actuatorValue;
	velocityActuatorPublisher.publish(msg);
}

void VelocityMeasuringProcess::stopMeasuring()
{
	MeasuringProcess::stopMeasuring();
}

void VelocityMeasuringProcess::laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg)
{
	std::unique_lock<std::mutex> lock{measuringMutex};
	if (measuringState != MeasuringState::MEASURING)
		return;
}

}