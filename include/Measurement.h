//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_MEASUREMENT_H
#define ACKERMANN2ACTUATOR_MEASUREMENT_H

#include <vector>

namespace a2a
{

struct Measurement
{
	double actuatorValue;
	double measuredValue;
};

using MeasurementSeries = std::vector<Measurement>;

}

#endif //ACKERMANN2ACTUATOR_MEASUREMENT_H
