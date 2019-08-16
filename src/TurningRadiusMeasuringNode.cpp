//
// Created by philipp on 15.08.19.
//

#include "../include/Common.h"
#include "../include/TurningRadiusMeasuringProcess.h"

int main(int argc, char ** argv)
{
	a2a::runExperimentNode<a2a::TurningRadiusMeasuringProcess>(argc, argv, "turning_radius_measuring_node");
	return 0;
}