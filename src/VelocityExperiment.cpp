//
// Created by philipp on 15.08.19.
//

#include "../include/VelocityExperiment.h"

namespace a2a
{

void VelocityExperiment::startExperiment(double actuatorValue)
{
	measurementPromise.set_value(0.0);
}

void VelocityExperiment::stopExperiment()
{

}

void VelocityExperiment::laserScanCallback(const sensor_msgs::LaserScanConstPtr & msg)
{
	
}

}