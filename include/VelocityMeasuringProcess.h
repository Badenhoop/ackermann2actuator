//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_VELOCITYMEASURINGPROCESS_H
#define ACKERMANN2ACTUATOR_VELOCITYMEASURINGPROCESS_H

#include "MeasuringProcess.h"

namespace a2a
{

class VelocityMeasuringProcess : public MeasuringProcess
{
public:
	VelocityMeasuringProcess() : MeasuringProcess("velocity_measuring")
	{}

private:
	double velocityActuatorValue;
	double accelerationDistance;
	double measuringDistance;

	double startDistance;
	ros::Time startTime;

	struct Measurement
	{
		float distance;
		ros::Time startScanTime;
	};

	std::vector<Measurement> measurements;

	void startMeasuring(float actuatorValue) override;

	void scanningDistanceState(const sensor_msgs::LaserScan &);

	void accelerationState(const sensor_msgs::LaserScan & scan);

	void measureState(const sensor_msgs::LaserScan & scan);

	float computeVelocity() const;
};

}

#endif //ACKERMANN2ACTUATOR_VELOCITYMEASURINGPROCESS_H
