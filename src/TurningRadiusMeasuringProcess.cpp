//
// Created by philipp on 15.08.19.
//

#include "../include/TurningRadiusMeasuringProcess.h"

namespace a2a
{

void TurningRadiusMeasuringProcess::startMeasuring(double actuatorValue)
{
	// keep steering angle neutral
	std_msgs::Float64 msg;
	msg.data = 0.0;
	steeringActuatorPublisher.publish(msg);

	// start driving
	msg.data = actuatorValue;
	velocityActuatorPublisher.publish(msg);
}

void TurningRadiusMeasuringProcess::stopMeasuring()
{
	MeasuringProcess::stopMeasuring();
}

void TurningRadiusMeasuringProcess::laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg)
{
	std::unique_lock<std::mutex> lock{measuringMutex};
	if (measuringState != MeasuringState::MEASURING)
		return;
}

}