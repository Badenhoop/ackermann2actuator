//
// Created by philipp on 26.08.19.
//

#ifndef SRC_MINRANGEWINDOWFILTER_H
#define SRC_MINRANGEWINDOWFILTER_H

#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

namespace a2a
{

/**
 * @param angle
 * @param lowerBound
 * @return Constrains a given angle to be in the interval [lowerBound, lowerBound + 2 * pi).
 */
double constrainAngle(double angle, double lowerBound);

class LaserScanMinRangeWindowFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
	virtual ~LaserScanMinRangeWindowFilter() = default;

	bool configure() override;

	bool update(const sensor_msgs::LaserScan & inputScan, sensor_msgs::LaserScan & filteredScan) override;

private:
	int windowSize;
	double changeInRangeThreshold;
	double rangeReplacementValue;
	double intensityReplacementValue;

	double lastWindowMinRange;
	double lastWindowMinAngle;

	bool firstUpdate;

	void updateWindow(const sensor_msgs::LaserScan & scan);

	void computeWindowedScan(const sensor_msgs::LaserScan & inputScan, sensor_msgs::LaserScan & windowedScan);
};

}

#endif //SRC_MINRANGEWINDOWFILTER_H
