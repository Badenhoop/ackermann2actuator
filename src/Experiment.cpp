//
// Created by philipp on 16.08.19.
//

#include "../include/Experiment.h"
#include "../include/Csv.h"
#include <iostream>
#include <std_msgs/Float64.h>

namespace a2a
{

const std::vector<std::string> Experiment::csvColumnNames = {"actuator_value", "measurement"};

Experiment::Experiment(std::string paramNamespace)
	: paramNamespace(std::move(paramNamespace))
{
	velocityActuatorPublisher = nh.advertise<std_msgs::Float64>("velocity_actuator", 1);
	steeringActuatorPublisher = nh.advertise<std_msgs::Float64>("steering_actuator", 1);
}

void Experiment::run()
{
	running = true;

	std::vector<double> actuatorValues;
	MeasurementSeries measurements;

	loadActuatorValues(actuatorValues);
	loadMeasurementSeries(measurements);

	for (std::size_t i = 0; i < actuatorValues.size() && running; ++i)
	{
		auto actuatorValue = actuatorValues[i];

		if (measurements.find(actuatorValue) != measurements.end())
		{
			std::cout << "Actuator value " << actuatorValue << " has already been measured.\n"
					  << "Starting a new experiment will overwrite it.\n";
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

		measurementPromise = std::promise<double>{};
		auto measurementFuture = measurementPromise.get_future();
		startExperiment(actuatorValue);
		try
		{
			auto measurement = measurementFuture.get();
			measurements[actuatorValue] = measurement;
		}
		catch (const ExperimentCancellation & )
		{
			break;
		}
	}

	safeMeasurementSeries(measurements);
}

void Experiment::cancel()
{
	running = false;
	try
	{
		stopExperiment();
		measurementPromise.set_exception(std::make_exception_ptr(ExperimentCancellation{}));
	}
	catch (const std::future_error & e)
	{}
}

void Experiment::loadActuatorValues(std::vector<double> & actuatorValues)
{
	ros::param::get("~" + paramNamespace + "/actuator_values", actuatorValues);
}

void Experiment::loadMeasurementSeries(MeasurementSeries & measurements)
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

void Experiment::safeMeasurementSeries(const Experiment::MeasurementSeries & measurements)
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

}