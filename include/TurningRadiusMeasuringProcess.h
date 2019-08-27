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
	ros::Time startTime;
	float velocityActuatorValue;
	float maxDistance;
	float minDistance;
	static const float DISTANCE_EPSILON;

	void startMeasuring(float actuatorValue) override;

	void scanningDistanceState(const sensor_msgs::LaserScan &);

	void findMaxDistanceState(const sensor_msgs::LaserScan & scan);

	void findMinDistanceState(const sensor_msgs::LaserScan & scan);
};

}

#endif //ACKERMANN2ACTUATOR_TURNINGRADIUSMEASURINGPROCESS_H
