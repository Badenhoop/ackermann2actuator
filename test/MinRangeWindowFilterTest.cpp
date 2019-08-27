//
// Created by philipp on 27.08.19.
//

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <filters/filter_chain.h>
#include "MinRangeWindowFilter.h"
#include <fstream>
#include <thread>
#include <chrono>

sensor_msgs::LaserScan generateScan(float minAngle,
									float maxAngle,
									std::size_t numSamples,
									float minRangeAngle,
									float minRange,
									float otherRange)
{
	sensor_msgs::LaserScan scan;
	scan.header.stamp = ros::Time::now();
	scan.header.frame_id = "laser";
	scan.angle_min = minAngle;
	scan.angle_max = maxAngle;
	scan.angle_increment = (maxAngle - minAngle) / numSamples;
	scan.time_increment = 0.01;
	scan.scan_time = 0.1;
	scan.range_min = 0.0;
	scan.range_max = 1.0;
	scan.ranges.resize(numSamples);
	scan.intensities.resize(numSamples);
	for (std::size_t i = 0; i < numSamples; ++i)
	{
		scan.ranges[i] = otherRange;
		scan.intensities[i] = 1.0;
	}
	auto minRangeIndex = std::size_t((minRangeAngle - minAngle) / scan.angle_increment);
	scan.ranges[minRangeIndex] = minRange;
	return scan;
}

double rad2deg(double rad)
{
	return rad * (180.0 / M_PI);
}

void printScan(const sensor_msgs::LaserScan & scan, std::ostream & stream)
{
	stream << "scan:\n";

	stream << "\tangular representation:\n";

	auto count = std::size_t(std::round((2.0 * M_PI) / scan.angle_increment));
	for (std::size_t i = 0; i < count; ++i)
	{
		auto angle = scan.angle_min + i * scan.angle_increment;
		auto constrainedAngle = a2a::constrainAngle(angle, 0.0);
		if (angle > scan.angle_max)
		{
			stream << "\t\t" << std::fixed << std::setprecision(2) << rad2deg(constrainedAngle) << ": -\n";
			continue;
		}

		auto range = scan.ranges[i];
		stream << "\t\t" << std::fixed << std::setprecision(2) << rad2deg(constrainedAngle) << ": "
		                 << std::fixed << std::setprecision(2) << range << "\n";
	}

	stream << "\tangle_min:\n\t\t" << rad2deg(scan.angle_min);
	stream << "\n\tangle_max:\n\t\t" << rad2deg(scan.angle_max);
	stream << "\n\tangle_increment:\n\t\t" << rad2deg(scan.angle_increment);
	stream << "\n\tranges:\n\t\t[";
	for (auto range : scan.ranges)
		stream << range << ", ";
	stream << "]\n\tintensities:\n\t\t[";
	for (auto intensity : scan.intensities)
		stream << intensity << ", ";
	stream << "]\n\n";
}

TEST(MinRangeWindowFilter, fixedMinimum)
{
	filters::FilterChain<sensor_msgs::LaserScan> filterChain("sensor_msgs::LaserScan");
	filterChain.configure("filter_chain");

	auto inputScan = generateScan(0, 2 * M_PI, 10, M_PI, 1, 2);
	sensor_msgs::LaserScan filteredScan;
	filterChain.update(inputScan, filteredScan);

	std::ofstream f{"/home/philipp/tmp/out.txt"};

	printScan(inputScan, f);
	printScan(filteredScan, f);
}

TEST(MinRangeWindowFilter, rotatingMinimum)
{

}

TEST(MinRangeWindowFilter, exceedThreshold)
{

}

int main(int argc, char ** argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "min_range_window_filter_test");
	ros::Time::init();
//	using namespace std::chrono_literals;
//	std::this_thread::sleep_for(10s);
	return RUN_ALL_TESTS();
}