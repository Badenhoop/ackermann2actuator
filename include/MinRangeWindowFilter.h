//
// Created by philipp on 26.08.19.
//

#ifndef SRC_MINRANGEWINDOWFILTER_H
#define SRC_MINRANGEWINDOWFILTER_H

#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

namespace a2a
{

class LaserScanMinRangeWindowFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
	virtual ~LaserScanMinRangeWindowFilter() = default;

	bool configure() override;

	bool update(const sensor_msgs::LaserScan & inputScan, sensor_msgs::LaserScan & filteredScan) override;

private:
	double windowAngle;
	double changeInRangeThreshold;
	double replacementValue;
};

}

#endif //SRC_MINRANGEWINDOWFILTER_H
