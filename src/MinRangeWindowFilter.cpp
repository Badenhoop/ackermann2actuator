//
// Created by philipp on 26.08.19.
//

#include "MinRangeWindowFilter.h"
#include "pluginlib/class_list_macros.h"
#include <cmath>

namespace a2a
{

bool LaserScanMinRangeWindowFilter::configure()
{
	lastWindowMinRange = 0;
	lastWindowCenterAngle = 0;
	firstUpdate = true;

	if (!getParam("window_angle", windowAngle) ||
	    !getParam("change_in_range_threshold", changeInRangeThreshold) ||
	    !getParam("range_replacement_value", rangeReplacementValue) ||
	    !getParam("intensity_replacement_value", intensityReplacementValue))
	{
		ROS_ERROR(
			"The parameters "
			"window_angle, change_in_range_threshold, range_replacement_value and replacement_value "
			"must be set to use this filter.");
		return false;
	}

	return true;
}

bool LaserScanMinRangeWindowFilter::update(const sensor_msgs::LaserScan & inputScan,
                                           sensor_msgs::LaserScan & filteredScan)
{
	if (firstUpdate)
	{
		updateWindow(inputScan);
		computeWindowedScan(inputScan, filteredScan);
		firstUpdate = false;
		return true;
	}

	computeWindowedScan(inputScan, filteredScan);
	updateWindow(filteredScan);
	return true;
}

void LaserScanMinRangeWindowFilter::updateWindow(const sensor_msgs::LaserScan & scan)
{
	auto it = std::min_element(scan.ranges.begin(), scan.ranges.end());
	auto index = std::distance(scan.ranges.begin(), it);
	lastWindowMinRange = *it;
	lastWindowCenterAngle = scan.angle_min + index * scan.angle_increment;
}

void LaserScanMinRangeWindowFilter::computeWindowedScan(const sensor_msgs::LaserScan & inputScan,
                                                        sensor_msgs::LaserScan & windowedScan)
{
	auto angleIncrement = (double) inputScan.angle_increment;
	auto size = (std::size_t) (windowAngle / angleIncrement);
	windowedScan.ranges.resize(size);
	windowedScan.intensities.resize(size);

	auto windowMinAngle = lastWindowCenterAngle - (windowAngle / 2.0);
	auto windowMaxAngle = lastWindowCenterAngle + (windowAngle / 2.0);

	for (std::size_t i = 0; i < size; ++i)
	{
		auto currWindowAngle = windowMinAngle + i * angleIncrement;
		// constrain currWindowAngle to be in the interval [inputScan.angle_min, inputScan.angle_min + 2 * pi)
		currWindowAngle = constrainAngle(currWindowAngle, inputScan.angle_min);

		// angle is not covered by the scan
		if (currWindowAngle > inputScan.angle_max)
		{
			windowedScan.ranges[i] = rangeReplacementValue;
			windowedScan.intensities[i] = intensityReplacementValue;
			continue;
		}

		auto currIndex = (std::size_t) ((currWindowAngle - inputScan.angle_min) / angleIncrement);
		auto currRange = inputScan.ranges[currIndex];
		auto currIntensity = inputScan.intensities[currIndex];

		// if change in range between two time steps is greater than given threshold
		if (std::abs(currRange - lastWindowMinRange) > changeInRangeThreshold)
		{
			windowedScan.ranges[i] = rangeReplacementValue;
			windowedScan.intensities[i] = intensityReplacementValue;
			continue;
		}

		windowedScan.ranges[i] = currRange;
		windowedScan.intensities[i] = currIntensity;
	}

	windowedScan.header.frame_id = inputScan.header.frame_id;
	windowedScan.header.stamp =
		inputScan.header.stamp + ros::Duration{
			((constrainAngle(windowMinAngle, inputScan.angle_min) - inputScan.angle_min) / angleIncrement)
			* inputScan.time_increment};
	windowedScan.angle_min = windowMinAngle;
	windowedScan.angle_max = windowMaxAngle;
	windowedScan.angle_increment = angleIncrement;
	windowedScan.time_increment = inputScan.time_increment;
	windowedScan.scan_time = inputScan.scan_time;
	windowedScan.range_min = inputScan.range_min;
	windowedScan.range_max = inputScan.range_max;
}

/**
 * @param angle
 * @param lowerBound
 * @return Constrains a given angle to be in the interval [lowerBound, lowerBound + 2 * pi).
 */
double LaserScanMinRangeWindowFilter::constrainAngle(double angle, double lowerBound)
{
	angle = std::fmod(angle - lowerBound, 2.0 * M_PI);
	if (angle < 0)
		angle += 2.0 * M_PI;
	return angle + lowerBound;
}

}

PLUGINLIB_EXPORT_CLASS(a2a::LaserScanMinRangeWindowFilter, filters::FilterBase<sensor_msgs::LaserScan>)