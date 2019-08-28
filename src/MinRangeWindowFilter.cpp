//
// Created by philipp on 26.08.19.
//

#include "MinRangeWindowFilter.h"
#include "pluginlib/class_list_macros.h"
#include <cmath>
#include "Utils.h"

namespace a2a
{

bool LaserScanMinRangeWindowFilter::configure()
{
	lastWindowMinRange = 0;
	lastWindowMinAngle = 0;
	firstUpdate = true;

	if (!getParam("window_size", windowSize) ||
	    !getParam("change_in_range_threshold", changeInRangeThreshold) ||
	    !getParam("range_replacement_value", rangeReplacementValue) ||
	    !getParam("intensity_replacement_value", intensityReplacementValue))
	{
		ROS_ERROR(
			"The parameters "
			"window_size, change_in_range_threshold, range_replacement_value and replacement_value "
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
	auto windowMinIndex = int(index) - (windowSize / 2);
	lastWindowMinRange = *it;
	lastWindowMinAngle = scan.angle_min + windowMinIndex * scan.angle_increment;
}

void LaserScanMinRangeWindowFilter::computeWindowedScan(const sensor_msgs::LaserScan & inputScan,
                                                        sensor_msgs::LaserScan & windowedScan)
{
	windowedScan.ranges.resize(windowSize);
	windowedScan.intensities.resize(windowSize);

	// constrain lastWindowMinAngle to be in the interval [inputScan.angle_min, inputScan.angle_min + 2 * pi)
	auto windowMinAngle = utils::constrainAngle(lastWindowMinAngle, inputScan.angle_min);
	auto windowMinIndex = std::size_t(std::round((windowMinAngle - inputScan.angle_min) / inputScan.angle_increment));
	auto incrementsPer2Pi = std::size_t(std::round(2.0 * M_PI / inputScan.angle_increment));

	for (std::size_t i = 0; i < windowSize; ++i)
	{
		// note: index could wrap over 2 * pi
		auto index = (windowMinIndex + i) % incrementsPer2Pi;
		// angle is not covered by the scan
		if (index >= inputScan.ranges.size())
		{
			windowedScan.ranges[i] = rangeReplacementValue;
			windowedScan.intensities[i] = intensityReplacementValue;
			continue;
		}

		auto range = inputScan.ranges[index];
		auto intensity = inputScan.intensities[index];

		// if change in range between two time steps is greater than given threshold
		if (std::abs(range - lastWindowMinRange) > changeInRangeThreshold)
		{
			windowedScan.ranges[i] = rangeReplacementValue;
			windowedScan.intensities[i] = intensityReplacementValue;
			continue;
		}

		windowedScan.ranges[i] = range;
		windowedScan.intensities[i] = intensity;
	}

	windowedScan.header.frame_id = inputScan.header.frame_id;
	windowedScan.header.stamp = inputScan.header.stamp + ros::Duration{windowMinIndex * inputScan.time_increment};
	windowedScan.angle_min = windowMinAngle;
	windowedScan.angle_max = windowMinAngle + (windowSize - 1) * inputScan.angle_increment;
	windowedScan.angle_increment = inputScan.angle_increment;
	windowedScan.time_increment = inputScan.time_increment;
	windowedScan.scan_time = inputScan.scan_time;
	windowedScan.range_min = inputScan.range_min;
	windowedScan.range_max = inputScan.range_max;
}

}

PLUGINLIB_EXPORT_CLASS(a2a::LaserScanMinRangeWindowFilter, filters::FilterBase<sensor_msgs::LaserScan>)