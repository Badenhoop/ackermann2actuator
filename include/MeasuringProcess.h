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
	using MeasurementSeries = std::map<double, double>;

	static const std::vector<std::string> csvColumnNames;

	void loadActuatorValues(std::vector<double> & actuatorValues);

	void loadMeasurementSeries(MeasurementSeries & measurements);

	void saveMeasurementSeries(const MeasurementSeries & measurements);

protected:
	std::atomic<bool> running{false};
	std::string paramNamespace;

	// Synchronized access on measurement result.
	// Unfortunately, std::promise isn't enough here.
	enum class MeasuringState
	{
		MEASURING, INTERRUPTED, FINISHED
	};
	MeasuringState measuringState{MeasuringState::FINISHED};
	double measuringResult;
	std::mutex measuringMutex;
	std::condition_variable measuringCondition;

	ros::NodeHandle nh;
	ros::Publisher velocityActuatorPublisher;
	ros::Publisher steeringActuatorPublisher;
	ros::Subscriber laserScanSubscriber;
	filters::FilterChain<sensor_msgs::LaserScan> filterChain;

	virtual void startMeasuring(double actuatorValue) = 0;

	virtual void stopMeasuring();

	virtual void laserScanCallback(const sensor_msgs::LaserScanConstPtr & scan) = 0;
};

}

#endif //ACKERMANN2ACTUATOR_MEASURINGPROCESS_H
