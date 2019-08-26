//
// Created by philipp on 15.08.19.
//

#include "VelocityMeasuringProcess.h"
#include "Common.h"

namespace a2a
{

void VelocityMeasuringProcess::startMeasuring(float actuatorValue)
{
	ros::param::get(paramNamespace + "/acceleration_distance", accelerationDistance);
	ros::param::get(paramNamespace + "/measuring_distance", measuringDistance);

	ROS_DEBUG_STREAM("acceleration distance: " << accelerationDistance);
	ROS_DEBUG_STREAM("measuring distance: " << measuringDistance);

	velocityActuatorValue = actuatorValue;
	startTime = ros::Time::now();
	measurements = std::vector<Measurement>{};

	// keep steering angle and velocity neutral
	std_msgs::Float64 msg;
	msg.data = 0.0;
	steeringActuatorPublisher.publish(msg);
	velocityActuatorPublisher.publish(msg);

	ROS_DEBUG_STREAM("Start scanning distance...");
	state = [&] (auto && ... args) { this->scanningDistanceState(args...); };
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
		state = [&] (auto && ... args) { this->accelerationState(args...); };
	}
}

void VelocityMeasuringProcess::accelerationState(const sensor_msgs::LaserScan & scan)
{
	float scannedDistance = getDistanceFromScan(scan);
	float travelledDistance = std::abs(startDistance - scannedDistance);
	ROS_DEBUG_STREAM("scanned distance: " << scannedDistance);
	ROS_DEBUG_STREAM("travelled distance: " << travelledDistance);
	if (travelledDistance >= accelerationDistance)
	{
		startTime = ros::Time::now();
		startDistance = scannedDistance;

		ROS_DEBUG_STREAM("Start measuring...");
		state = [&] (auto && ... args) { this->measureState(args...); };
	}
}

void VelocityMeasuringProcess::measureState(const sensor_msgs::LaserScan & scan)
{
	auto now = ros::Time::now();
	float scannedDistances = getDistanceFromScan(scan);
	measurements.emplace_back(Measurement{now, scannedDistances});
	float travelledDistance = std::abs(startDistance - scannedDistances);
	ROS_DEBUG_STREAM("scanned distance: " << scannedDistances);
	ROS_DEBUG_STREAM("travelled distance: " << travelledDistance);
	if (travelledDistance >= measuringDistance)
	{
		measuringResult = computeVelocity();
		ROS_DEBUG_STREAM("Finished measuring speed.");
		ROS_DEBUG_STREAM("Measured speed: " << measuringResult);
		measuringState = MeasuringState::FINISHED;
		measuringCondition.notify_all();
		stopMeasuring();
	}
}

float VelocityMeasuringProcess::getDistanceFromScan(const sensor_msgs::LaserScan & scan)
{
	return *std::min_element(scan.ranges.begin(), scan.ranges.end());
}

float VelocityMeasuringProcess::computeVelocity() const
{
	if (measurements.size() < 2)
	{
		ROS_INFO_STREAM("Faulty measurement!");
		return 0;
	}

	double velocity = 0;
	for (std::size_t i = 1; i < measurements.size(); ++i)
	{
		double deltaTime = std::abs(measurements[i].time.toSec() - measurements[i - 1].time.toSec());
		double distance = std::abs(measurements[i].distance - measurements[i - 1].distance);
		velocity += distance / deltaTime;
	}
	return velocity / (measurements.size() - 1);
}

}