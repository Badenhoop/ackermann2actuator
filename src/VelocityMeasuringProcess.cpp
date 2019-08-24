//
// Created by philipp on 15.08.19.
//

#include "../include/VelocityMeasuringProcess.h"
#include "../include/Common.h"

namespace a2a
{

void VelocityMeasuringProcess::startMeasuring(float actuatorValue)
{
	ros::param::get("~" + paramNamespace + "/acceleration_distance", accelerationDistance);
	ros::param::get("~" + paramNamespace + "/measuring_distance", measuringDistance);

	ROS_DEBUG_STREAM("acceleration distance: " << accelerationDistance);
	ROS_DEBUG_STREAM("measuring distance: " << measuringDistance);

	velocityActuatorValue = actuatorValue;
	startTime = ros::Time::now();

	if (velocityActuatorValue >= 0)
		driveMode = DriveMode::FORWARD;
	else
		driveMode = DriveMode::BACKWARD;

	ROS_DEBUG_STREAM("Start scanning distance...");
	state = [&] (auto && ... args) { this->scanningDistanceState(args...); };

	// keep steering angle neutral
	std_msgs::Float64 msg;
	msg.data = 0.0;
	steeringActuatorPublisher.publish(msg);
}

void VelocityMeasuringProcess::laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg)
{
	std::unique_lock<std::mutex> lock{measuringMutex};
	if (measuringState != MeasuringState::MEASURING)
		return;

	sensor_msgs::LaserScan filteredMsg;
	filterChain.update(*msg, filteredMsg);

	state(filteredMsg);
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
	float distanceToWall = getDistanceFromScan(scan);
	float travelledDistance = std::abs(startDistance - distanceToWall);
	ROS_DEBUG_STREAM("distance to wall: " << distanceToWall);
	ROS_DEBUG_STREAM("travelled distance: " << travelledDistance);
	if (travelledDistance >= accelerationDistance)
	{
		startTime = ros::Time::now();
		startDistance = distanceToWall;

		ROS_DEBUG_STREAM("Start measuring...");
		state = [&] (auto && ... args) { this->measureState(args...); };
	}
}

void VelocityMeasuringProcess::measureState(const sensor_msgs::LaserScan & scan)
{
	float distanceToWall = getDistanceFromScan(scan);
	float travelledDistance = std::abs(startDistance - distanceToWall);
	ROS_DEBUG_STREAM("distance to wall: " << distanceToWall);
	ROS_DEBUG_STREAM("travelled distance: " << travelledDistance);
	if (travelledDistance >= measuringDistance)
	{
		auto deltaTime = ros::Time::now() - startTime;
		// v = s / t
		measuringResult = travelledDistance / deltaTime.toSec();
		ROS_DEBUG_STREAM("Finished measuring speed.");
		ROS_DEBUG_STREAM("Measured speed: " << measuringResult);
		measuringState = MeasuringState::FINISHED;
		measuringCondition.notify_all();
		stopMeasuring();
	}
}

}