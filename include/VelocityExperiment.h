//
// Created by philipp on 15.08.19.
//

#ifndef ACKERMANN2ACTUATOR_VELOCITYEXPERIMENT_H
#define ACKERMANN2ACTUATOR_VELOCITYEXPERIMENT_H

#include "Experiment.h"

namespace a2a
{

class VelocityExperiment : public Experiment
{
public:
	VelocityExperiment() : Experiment("velocity_measuring")
	{}

protected:
	void startExperiment(double actuatorValue) override;

	void stopExperiment() override;
};

}

#endif //ACKERMANN2ACTUATOR_VELOCITYEXPERIMENT_H
