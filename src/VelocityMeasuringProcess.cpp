//
// Created by philipp on 15.08.19.
//

#include "VelocityMeasuringProcess.h"
#include "Common.h"

namespace a2a
{

void VelocityMeasuringProcess::startMeasuring(float actuatorValue)
{
	MeasuringProcess::startMeasuring(actuatorValue);

	ros::param::get(paramNamespace + "/acceleration_distance", accelerationDistance);
	ROS_DEBUG_STREAM("acceleration distance: " << accelerationDistance);

	ros::param::get(paramNamespace + "/measuring_distance", measuringDistance);
	ROS_DEBUG_STREAM("measuring distance: " << measuringDistance);

	ros::param::get(paramNamespace + "/safety_distance", safetyDistance);
	ROS_DEBUG_STREAM("safety distance: " << safetyDistance);

	velocityActuatorValue = actuatorValue;
	startTime = ros::Time::now();
	measurements = std::vector<Measurement>{};

	// keep steering angle and velocity neutral
	std_msgs::Float64 msg;
	msg.data = 0.0;
	steeringActuatorPublisher.publish(msg);
	velocityActuatorPublisher.publish(msg);

	ROS_DEBUG_STREAM("Start scanning distance...");
	measuringState = [&] (auto && ... args) { this->scanningDistanceState(args...); };
}

void VelocityMeasuringProcess::scanningDistanceState(const sensor_msgs::LaserScan & scan)
{
	auto now = ros::Time::now();
	if (now - startTime >= ros::Duration{1})
	{
		startTime = now;
		startDistance = getDistanceFromScan(scan);

		// start driving
		std_msgs::Float64 msg;
		msg.data = velocityActuatorValue;
		velocityActuatorPublisher.publish(msg);

		ROS_DEBUG_STREAM("Start accelerating...");
		measuringState = [&] (auto && ... args) { this->accelerationState(args...); };
	}
}

void VelocityMeasuringProcess::accelerationState(const sensor_msgs::LaserScan & scan)
{
	auto scannedDistance = getDistanceFromScan(scan);
	auto travelledDistance = std::abs(startDistance - scannedDistance);
	ROS_DEBUG_STREAM("scanned distance: " << scannedDistance);
	ROS_DEBUG_STREAM("travelled distance: " << travelledDistance);
	if (travelledDistance >= accelerationDistance)
	{
		startTime = ros::Time::now();
		startDistance = scannedDistance;

		ROS_DEBUG_STREAM("Start measuring...");
		measuringState = [&] (auto && ... args) { this->measureState(args...); };
	}
}

void VelocityMeasuringProcess::measureState(const sensor_msgs::LaserScan & scan)
{
	auto scannedDistances = getDistanceFromScan(scan);
	measurements.emplace_back(Measurement{scannedDistances, scan.header.stamp});
	float travelledDistance = std::abs(startDistance - scannedDistances);
	ROS_DEBUG_STREAM("scanned distance: " << scannedDistances);
	ROS_DEBUG_STREAM("travelled distance: " << travelledDistance);
	if (travelledDistance >= measuringDistance)
	{
		finishMeasuring(computeVelocity());
	}
}

float VelocityMeasuringProcess::computeVelocity() const
{
	if (measurements.size() < 2)
	{
		throw BadMeasuringException{"Too few collected scans!"};
	}

	double velocity = 0;
	for (std::size_t i = 1; i < measurements.size(); ++i)
	{
		double distance = std::abs(measurements[i].distance - measurements[i - 1].distance);
		double deltaTime = std::abs(measurements[i].startScanTime.toSec() - measurements[i - 1].startScanTime.toSec());
		velocity += distance / deltaTime;
	}
	return velocity / (measurements.size() - 1);
}

float VelocityMeasuringProcess::getDistanceFromScan(const sensor_msgs::LaserScan & scan)
{
	auto distance = MeasuringProcess::getDistanceFromScan(scan);
	if (distance < safetyDistance)
	{
		throw BadMeasuringException{"Distance below safety distance."};
	}
	return distance;
}

}