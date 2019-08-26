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
	virtual ~LaserScanMinRangeWindowFilter() {}

	bool configure();

	bool update(const sensor_msgs::LaserScan & inputScan, sensor_msgs::LaserScan & filteredScan);

private:
	float windowAngle;
	float changeInRangeThreshold;
	float replacementValue;
};

}

#endif //SRC_MINRANGEWINDOWFILTER_H
