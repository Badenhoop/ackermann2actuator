//
// Created by philipp on 15.08.19.
//

#include "TurningRadiusMeasuringProcess.h"

namespace a2a
{

const float TurningRadiusMeasuringProcess::DISTANCE_EPSILON = 0.1;

void TurningRadiusMeasuringProcess::startMeasuring(float actuatorValue)
{
	MeasuringProcess::startMeasuring(actuatorValue);

	ros::param::get(paramNamespace + "/velocity_actuator_value", velocityActuatorValue);
	ROS_DEBUG_STREAM("velocity actuator value: " << velocityActuatorValue);

	startTime = ros::Time::now();
	maxDistance = std::numeric_limits<float>::min();
	minDistance = std::numeric_limits<float>::max();

	std_msgs::Float64 msg;
	msg.data = 0.0;
	velocityActuatorPublisher.publish(msg);
	msg.data = actuatorValue;
	steeringActuatorPublisher.publish(msg);

	ROS_DEBUG_STREAM("Start scanning distance...");
	measuringState = [&] (auto && ... args) { this->scanningDistanceState(args...); };
}

void TurningRadiusMeasuringProcess::scanningDistanceState(const sensor_msgs::LaserScan &)
{
	auto now = ros::Time::now();
	if (now - startTime >= ros::Duration{1})
	{
		startTime = now;

		// start driving
		std_msgs::Float64 msg;
		msg.data = velocityActuatorValue;
		velocityActuatorPublisher.publish(msg);

		ROS_DEBUG_STREAM("Start finding maximum distance...");
		measuringState = [&] (auto && ... args) { this->findMaxDistanceState(args...); };
	}
}

void TurningRadiusMeasuringProcess::findMaxDistanceState(const sensor_msgs::LaserScan & scan)
{
	auto distance = getDistanceFromScan(scan);
	maxDistance = std::max(maxDistance, distance);
	if (std::abs(distance - maxDistance) > DISTANCE_EPSILON)
	{
		ROS_DEBUG_STREAM("Start finding minimum distance...");
		measuringState = [&] (auto && ... args) { this->findMinDistanceState(args...); };
	}
}

void TurningRadiusMeasuringProcess::findMinDistanceState(const sensor_msgs::LaserScan & scan)
{
	auto distance = getDistanceFromScan(scan);
	minDistance = std::min(minDistance, distance);
	if (std::abs(distance - minDistance) > DISTANCE_EPSILON)
	{
		float turningRadius = std::abs(maxDistance - minDistance);
		finishMeasuring(turningRadius);
	}
}

}