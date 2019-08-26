//
// Created by philipp on 26.08.19.
//

#include "MinRangeWindowFilter.h"
#include "pluginlib/class_list_macros.h"

namespace a2a
{

bool LaserScanMinRangeWindowFilter::configure()
{
	// TODO implementation
	return false;
}

bool LaserScanMinRangeWindowFilter::update(const sensor_msgs::LaserScan & inputScan, sensor_msgs::LaserScan & filteredScan)
{
	// TODO implementation
	return false;
}

}

PLUGINLIB_EXPORT_CLASS(a2a::LaserScanMinRangeWindowFilter, filters::FilterBase<sensor_msgs::LaserScan>)