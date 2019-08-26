//
// Created by philipp on 15.08.19.
//

#include "TurningRadiusMeasuringProcess.h"

namespace a2a
{

void TurningRadiusMeasuringProcess::startMeasuring(float actuatorValue)
{
	startTime = ros::Time::now();

	std_msgs::Float64 msg;
	msg.data = 0.0;
	velocityActuatorPublisher.publish(msg);
	msg.data = actuatorValue;
	steeringActuatorPublisher.publish(msg);

	ROS_DEBUG_STREAM("Start scanning distance...");
	state = [&] (auto && ... args) { this->scanningDistanceState(args...); };
}

void TurningRadiusMeasuringProcess::scanningDistanceState(const sensor_msgs::LaserScan &)
{

}

}