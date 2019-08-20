//
// Created by philipp on 16.08.19.
//

#include "../include/MeasuringProcess.h"
#include "../include/Csv.h"
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
	filterChain.configure(this->paramNamespace + "_filter_chain");
}

void MeasuringProcess::run()
{
	running = true;

	std::vector<float> actuatorValues;
	MeasurementSeries measurements;

	loadActuatorValues(actuatorValues);
	loadMeasurementSeries(measurements);

	//ROS_DEBUG_STREAM("actuatorValues.size(): " << actuatorValues.size());
	//ROS_DEBUG_STREAM("running: " << running);

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
		measuringState = MeasuringState::MEASURING;
		startMeasuring(actuatorValue);

		ROS_DEBUG_STREAM("waiting");

		while (measuringState == MeasuringState::MEASURING)
		{
			measuringCondition.wait(lock);
		}

		if (measuringState == MeasuringState::INTERRUPTED)
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
	if (measuringState == MeasuringState::MEASURING)
	{
		measuringState = MeasuringState::INTERRUPTED;
		measuringCondition.notify_all();
		stopMeasuring();
	}
}

void MeasuringProcess::loadActuatorValues(std::vector<float> & actuatorValues)
{
	ros::param::get("~" + paramNamespace + "/actuator_values", actuatorValues);
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

void MeasuringProcess::stopMeasuring()
{
	std_msgs::Float64 msg;
	msg.data = 0.0;
	velocityActuatorPublisher.publish(msg);
	steeringActuatorPublisher.publish(msg);
}

}
