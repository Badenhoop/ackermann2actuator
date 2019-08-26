//
// Created by philipp on 15.08.19.
//

#include "Common.h"
#include "VelocityMeasuringProcess.h"

int main(int argc, char ** argv)
{
	a2a::runExperimentNode<a2a::VelocityMeasuringProcess>(argc, argv, "velocity_measuring_node");
	return 0;
}