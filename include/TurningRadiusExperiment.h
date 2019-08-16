//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_TURNINGRADIUSEXPERIMENT_H
#define ACKERMANN2ACTUATOR_TURNINGRADIUSEXPERIMENT_H

#include "Experiment.h"

namespace a2a
{

class TurningRadiusExperiment : public Experiment
{
public:
	TurningRadiusExperiment() : Experiment("steering_angle_measuring")
	{}

protected:
	void startExperiment(double actuatorValue) override;
	void stopExperiment() override;
};

}

#endif //ACKERMANN2ACTUATOR_TURNINGRADIUSEXPERIMENT_H
