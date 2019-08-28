//
// Created by philipp on 16.08.19.
//

#include "MeasuringProcess.h"
#include "Csv.h"
#include <iostream>
#include <std_msgs/Float64.h>

namespace a2a
{

const std::vector<std::string> MeasuringProcess::csvColumnNames = {"actuator_value", "measurement"};

MeasuringProcess::MeasuringProcess(std::string paramNamespace)
	: paramNamespace(std::move(paramNamespace))
	, filterChain("sensor_msgs::LaserScan")
{
	velocityActuatorPublisher = nh.advertise<std_msgs::Float64>("velocity_actuator", 1);
	steeringActuatorPublisher = nh.advertise<std_msgs::Float64>("steering_actuator", 1);
	laserScanSubscriber = nh.subscribe("scan", 10, &MeasuringProcess::laserScanCallback, this);
}

void MeasuringProcess::run()
{
	running = true;

	std::vector<float> actuatorValues;
	MeasurementSeries measurements;

	loadActuatorValues(actuatorValues);
	loadMeasurementSeries(measurements);

	for (std::size_t i = 0; i < actuatorValues.size() && running; ++i)
	{
		auto actuatorValue = actuatorValues[i];

		if (measurements.find(actuatorValue) != measurements.end())
		{
			std::cout << "Actuator value " << actuatorValue << " has already been measured.\n"
					  << "Starting a new measuring process will overwrite it.\n";
		}
		else
		{
			std::cout << "Starting new experiment with actuator value " << actuatorValue << ".\n";
		}

		std::cout << "(y)es / (n)o / (q)uit\n";

		std::string input;
		while (input != "y" && input != "n" && input != "q" && running)
			std::cin >> input;

		if (input == "q" || !running)
		{
			running = false;
			break;
		}
		else if (input == "n")
		{
			continue;
		}

		std::unique_lock<std::mutex> lock{measuringMutex};
		processState = ProcessState::MEASURING;
		startMeasuring(actuatorValue);

		ROS_DEBUG_STREAM("waiting");

		while (processState == ProcessState::MEASURING)
		{
			measuringCondition.wait(lock);
		}

		if (processState == ProcessState::INTERRUPTED)
		{
			running = false;
			break;
		}

		measurements[actuatorValue] = measuringResult;
		std::cout << "Measuring process for actuator value " << actuatorValue << " finished.\n";
	}

	saveMeasurementSeries(measurements);
}

void MeasuringProcess::cancel()
{
	running = false;

	std::unique_lock<std::mutex> lock{measuringMutex};
	if (processState == ProcessState::MEASURING)
	{
		processState = ProcessState::INTERRUPTED;
		measuringCondition.notify_all();
		stop();
	}
}

void MeasuringProcess::loadActuatorValues(std::vector<float> & actuatorValues)
{
	ros::param::get(paramNamespace + "/actuator_values", actuatorValues);
}

void MeasuringProcess::loadMeasurementSeries(MeasurementSeries & measurements)
{
	const std::string param{"~input_measurement_series"};
	std::string filename;

	if (!ros::param::has(param))
		return;

	ros::param::get(param, filename);
	csv::IStream stream{filename, csvColumnNames};
	std::vector<csv::Item> items;
	while (stream >> items)
	{
		auto actuatorValue = items[0].getDouble();
		auto measurement = items[1].getDouble();
		measurements[actuatorValue] = measurement;
	}
}

void MeasuringProcess::saveMeasurementSeries(const MeasuringProcess::MeasurementSeries & measurements)
{
	std::string filename;
	ros::param::get("~output_measurement_series", filename);
	csv::OStream stream{filename, csvColumnNames};
	for (const auto & pair : measurements)
	{
		auto actuatorValue = pair.first;
		auto measurement = pair.second;
		stream << std::vector<csv::Item>{actuatorValue, measurement};
	}
}

void MeasuringProcess::stop()
{
	std_msgs::Float64 msg;
	msg.data = 0.0;
	velocityActuatorPublisher.publish(msg);
	steeringActuatorPublisher.publish(msg);
}

void MeasuringProcess::laserScanCallback(const sensor_msgs::LaserScanConstPtr & scan)
{
	std::unique_lock<std::mutex> lock{measuringMutex};
	if (processState != ProcessState::MEASURING)
		return;

	sensor_msgs::LaserScan filteredScan;
	filterChain.update(*scan, filteredScan);

	try
	{
		measuringState(filteredScan);
	}
	catch (const BadMeasuringException & e)
	{
		ROS_ERROR_STREAM("Bad measurement! Please try again. The reason: " << e.what());
		finishMeasuring(0);
	}
}

float MeasuringProcess::getDistanceFromScan(const sensor_msgs::LaserScan & scan)
{
	auto distance = *std::min_element(scan.ranges.begin(), scan.ranges.end());
	if (distance == std::numeric_limits<float>::infinity())
	{
		throw BadMeasuringException{"Scanned distance: infinity."};
	}
	return distance;
}

void MeasuringProcess::startMeasuring(float actuatorValue)
{
	filterChain.configure(this->paramNamespace + "/filter_chain");
}

void MeasuringProcess::finishMeasuring(float result)
{
	ROS_DEBUG_STREAM("Measuring result: " << result);
	measuringResult = result;
	processState = ProcessState::FINISHED;
	measuringCondition.notify_all();
	stop();
}

}
