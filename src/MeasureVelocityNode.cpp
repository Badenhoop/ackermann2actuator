//
// Created by philipp on 15.08.19.
//

#include "../include/Common.h"
#include "../include/VelocityExperiment.h"

int main(int argc, char ** argv)
{
	a2a::runExperimentNode<a2a::VelocityExperiment>(argc, argv, "measure_velocity_node");
	return 0;
}