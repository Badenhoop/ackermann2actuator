//
// Created by philipp on 16.08.19.
//

#ifndef ACKERMANN2ACTUATOR_EXPERIMENT_H
#define ACKERMANN2ACTUATOR_EXPERIMENT_H

#include <future>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace a2a
{

class Experiment
{
public:
	explicit Experiment(std::string paramNamespace);

	virtual ~Experiment() = default;

	void run();

	bool isRunning() const
	{ return running; }

	void cancel();

private:
	// maps from actuator value to measurement
	using MeasurementSeries = std::map<double, double>;

	static const std::vector<std::string> csvColumnNames;

	std::string paramNamespace;
	std::atomic<bool> running{false};

	void loadActuatorValues(std::vector<double> & actuatorValues);

	void loadMeasurementSeries(MeasurementSeries & measurements);

	void safeMeasurementSeries(const MeasurementSeries & measurements);

protected:
	std::promise<double> measurementPromise;
	ros::NodeHandle nh;
	ros::Publisher velocityActuatorPublisher;
	ros::Publisher steeringActuatorPublisher;
	ros::Subscriber laserScanSubscriber;

	virtual void startExperiment(double actuatorValue) = 0;

	virtual void stopExperiment() = 0;

	virtual void laserScanCallback(const sensor_msgs::LaserScanConstPtr & scan) = 0;
};

class ExperimentCancellation : public std::exception
{
};

}

#endif //ACKERMANN2ACTUATOR_EXPERIMENT_H
