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
	double startDistance;
	ros::Time startTime;
	double accelerationDistance;
	double measuringDistance;

	void startMeasuring(double actuatorValue) override;

	void stopMeasuring() override;

	void laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg) override;
};

}

#endif //ACKERMANN2ACTUATOR_VELOCITYMEASURINGPROCESS_H
