//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_TURNINGRADIUSMEASURINGPROCESS_H
#define ACKERMANN2ACTUATOR_TURNINGRADIUSMEASURINGPROCESS_H

#include "MeasuringProcess.h"

namespace a2a
{

class TurningRadiusMeasuringProcess : public MeasuringProcess
{
public:
	TurningRadiusMeasuringProcess() : MeasuringProcess("turning_radius_measuring")
	{}

private:
	void startMeasuring(float actuatorValue) override;

	void laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg) override;
};

}

#endif //ACKERMANN2ACTUATOR_TURNINGRADIUSMEASURINGPROCESS_H
