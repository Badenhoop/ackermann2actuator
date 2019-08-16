//
// Created by philipp on 15.08.19.
//

#include "../include/Common.h"
#include "../include/TurningRadiusExperiment.h"

int main(int argc, char ** argv)
{
	a2a::runExperimentNode<a2a::TurningRadiusExperiment>(argc, argv, "measure_turning_radius_node");
	return 0;
}