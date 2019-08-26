//
// Created by philipp on 26.08.19.
//

#include "MinRangeWindowFilter.h"
#include "pluginlib/class_list_macros.h"

namespace a2a
{

bool LaserScanMinRangeWindowFilter::configure()
{
	if (!getParam("window_angle", windowAngle) ||
	    !getParam("change_in_range_threshold", changeInRangeThreshold) ||
	    !getParam("replacement_value", replacementValue))
	{
		ROS_ERROR(
			"The parameters window_angle, change_in_range_threshold and replacement_value must be set to use this filter.");
		return false;
	}

	return true;
}

bool
LaserScanMinRangeWindowFilter::update(const sensor_msgs::LaserScan & inputScan, sensor_msgs::LaserScan & filteredScan)
{
	// TODO implementation
	return false;
}

}

PLUGINLIB_EXPORT_CLASS(a2a::LaserScanMinRangeWindowFilter, filters::FilterBase<sensor_msgs::LaserScan>)