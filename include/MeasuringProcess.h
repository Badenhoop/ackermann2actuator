//
// Created by philipp on 16.08.19.
//

#ifndef ACKERMANN2ACTUATOR_MEASURINGPROCESS_H
#define ACKERMANN2ACTUATOR_MEASURINGPROCESS_H

#include <mutex>
#include <condition_variable>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "filters/filter_chain.h"
#include <std_msgs/Float64.h>

namespace a2a
{

class MeasuringProcess
{
public:
	explicit MeasuringProcess(std::string paramNamespace);

	virtual ~MeasuringProcess() = default;

	void run();

	bool isRunning() const
	{ return running; }

	void cancel();

private:
	// maps from actuator value to measurement
	using MeasurementSeries = std::map<float, double>;

	// Synchronized access on measurement result.
	// Unfortunately, std::promise isn't enough here.
	enum class ProcessState
	{
		MEASURING, INTERRUPTED, FINISHED
	};

	ProcessState processState{ProcessState::FINISHED};
	float measuringResult;
	std::mutex measuringMutex;
	std::condition_variable measuringCondition;

	static const std::vector<std::string> csvColumnNames;

	void loadActuatorValues(std::vector<float> & actuatorValues);

	void loadMeasurementSeries(MeasurementSeries & measurements);

	void saveMeasurementSeries(const MeasurementSeries & measurements);

	void stop();

protected:
	std::atomic<bool> running{false};
	std::string paramNamespace;
	std::function<void(const sensor_msgs::LaserScan &)> measuringState;

	ros::NodeHandle nh;
	ros::Publisher velocityActuatorPublisher;
	ros::Publisher steeringActuatorPublisher;
	ros::Subscriber laserScanSubscriber;
	filters::FilterChain<sensor_msgs::LaserScan> filterChain;

	virtual void startMeasuring(float actuatorValue) = 0;

	void laserScanCallback(const sensor_msgs::LaserScanConstPtr & scan);

	float getDistanceFromScan(const sensor_msgs::LaserScan & scan);

	void finishMeasuring(float result);
};

}

#endif //ACKERMANN2ACTUATOR_MEASURINGPROCESS_H
