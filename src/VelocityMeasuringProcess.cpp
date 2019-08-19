//
// Created by philipp on 15.08.19.
//

#include <std_msgs/Float64.h>
#include "../include/VelocityMeasuringProcess.h"

namespace a2a
{

void VelocityMeasuringProcess::startMeasuring(double actuatorValue)
{
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

}

}