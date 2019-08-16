//
// Created by philipp on 15.08.19.
//

#include "../include/VelocityMeasuringProcess.h"

namespace a2a
{

void VelocityMeasuringProcess::startMeasuring(double actuatorValue)
{
	measurementPromise.set_value(0.0);
}

void VelocityMeasuringProcess::stopMeasuring()
{

}

void VelocityMeasuringProcess::laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg)
{
	
}

}