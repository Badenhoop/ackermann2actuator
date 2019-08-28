//
// Created by philipp on 28.08.19.
//

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

#include <cmath>

namespace a2a
{
namespace utils
{

/**
 * @param angle
 * @param lowerBound
 * @return Constrains a given angle to be in the interval [lowerBound, lowerBound + 2 * pi).
 */
double constrainAngle(double angle, double lowerBound)
{
	angle = std::fmod(angle - lowerBound, 2.0 * M_PI);
	if (angle < 0)
		angle += 2.0 * M_PI;
	return angle + lowerBound;
}

double rad2deg(double rad)
{
	return rad * (180.0 / M_PI);
}

double deg2rad(double deg)
{
	return deg * (M_PI / 180.0);
}

}
}

#endif //SRC_UTILS_H
