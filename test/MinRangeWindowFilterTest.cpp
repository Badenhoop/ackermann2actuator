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

constexpr double EPSILON = 0.0001;
constexpr float inf = std::numeric_limits<float>::infinity();

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
	scan.angle_increment = (maxAngle - minAngle) / (numSamples - 1);
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
	auto minRangeIndex = std::size_t(std::round((minRangeAngle - minAngle) / scan.angle_increment));
	scan.ranges[minRangeIndex] = minRange;
	return scan;
}

double rad2deg(double rad)
{
	return rad * (180.0 / M_PI);
}

double deg2rad(double deg)
{
	return deg * (M_PI / 180.0);
}

void compareFloats(float f1, float f2)
{
	// in case both are inf: do nothing which is success
	if (!std::isinf(f1) || !std::isinf(f2))
		EXPECT_NEAR(f1, f2, EPSILON);
}

void compareScans(const sensor_msgs::LaserScan & scan1, const sensor_msgs::LaserScan & scan2)
{
	EXPECT_NEAR(scan1.header.stamp.toSec(), scan2.header.stamp.toSec(), EPSILON);
	EXPECT_EQ(scan1.header.frame_id, scan2.header.frame_id);
	EXPECT_NEAR(scan1.angle_min, scan2.angle_min, EPSILON);
	EXPECT_NEAR(scan1.angle_max, scan2.angle_max, EPSILON);
	EXPECT_NEAR(scan1.angle_increment, scan2.angle_increment, EPSILON);
	EXPECT_NEAR(scan1.scan_time, scan2.scan_time, EPSILON);
	EXPECT_EQ(scan1.range_min, scan2.range_min);
	EXPECT_EQ(scan1.range_max, scan2.range_max);
	EXPECT_EQ(scan1.ranges.size(), scan2.ranges.size());
	EXPECT_EQ(scan1.intensities.size(), scan2.intensities.size());
	for (std::size_t i = 0; i < scan1.ranges.size(); ++i)
		compareFloats(scan1.ranges[i], scan2.ranges[i]);
	for (std::size_t i = 0; i < scan1.intensities.size(); ++i)
		compareFloats(scan1.intensities[i], scan2.intensities[i]);
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

TEST(MinRangeWindowFilter, rotatingMinimum)
{
	filters::FilterChain<sensor_msgs::LaserScan> filterChain("sensor_msgs::LaserScan");
	filterChain.configure("rotating_minimum");

	sensor_msgs::LaserScan inputScan;
	sensor_msgs::LaserScan filteredScan;
	sensor_msgs::LaserScan shouldScan;

	inputScan = generateScan(deg2rad(0), deg2rad(315), 8, deg2rad(180), 1, 2);
	filterChain.update(inputScan, filteredScan);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{3 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(135);
	shouldScan.angle_max = deg2rad(225);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, 1.f, 2.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);

	inputScan = generateScan(deg2rad(0), deg2rad(315), 8, deg2rad(225), 1, 2);
	filterChain.update(inputScan, filteredScan);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{3 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(135);
	shouldScan.angle_max = deg2rad(225);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, 2.f, 1.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);

	inputScan = generateScan(deg2rad(0), deg2rad(315), 8, deg2rad(270), 1, 2);
	filterChain.update(inputScan, filteredScan);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{4 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(180);
	shouldScan.angle_max = deg2rad(270);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, 2.f, 1.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);
}

TEST(MinRangeWindowFilter, non360Scan)
{
	filters::FilterChain<sensor_msgs::LaserScan> filterChain("sensor_msgs::LaserScan");
	filterChain.configure("non_360_scan");

	sensor_msgs::LaserScan inputScan;
	sensor_msgs::LaserScan filteredScan;
	sensor_msgs::LaserScan shouldScan;

	inputScan = generateScan(deg2rad(0), deg2rad(270), 7, deg2rad(270), 1, 2);
	filterChain.update(inputScan, filteredScan);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{4 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(180);
	shouldScan.angle_max = deg2rad(360);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, 2.f, 1.f, inf, 2.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f, inf, 1.f};

	compareScans(filteredScan, shouldScan);

	inputScan = generateScan(deg2rad(0), deg2rad(270), 7, deg2rad(0), 1, 2);
	filterChain.update(inputScan, filteredScan);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{4 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(180);
	shouldScan.angle_max = deg2rad(360);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, 2.f, 2.f, inf, 1.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f, inf, 1.f};

	compareScans(filteredScan, shouldScan);

	inputScan = generateScan(deg2rad(0), deg2rad(270), 7, deg2rad(45), 1, 2);
	filterChain.update(inputScan, filteredScan);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{6 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(270);
	shouldScan.angle_max = deg2rad(450);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, inf, 2.f, 1.f, 2.f};
	shouldScan.intensities = std::vector<float>{1.f, inf, 1.f, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);
}

TEST(MinRangeWindowFilter, exceedChangeInRangeThreshold)
{
	filters::FilterChain<sensor_msgs::LaserScan> filterChain("sensor_msgs::LaserScan");
	filterChain.configure("exceed_change_in_range_threshold");

	sensor_msgs::LaserScan inputScan;
	sensor_msgs::LaserScan filteredScan;
	sensor_msgs::LaserScan shouldScan;

	std::ofstream f{"/home/philipp/tmp/out.txt"};

	inputScan = generateScan(deg2rad(0), deg2rad(315), 8, deg2rad(180), 3, 4);
	filterChain.update(inputScan, filteredScan);
	printScan(filteredScan, f);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{3 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(135);
	shouldScan.angle_max = deg2rad(225);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{4.f, 3.f, 4.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);

	inputScan = generateScan(deg2rad(0), deg2rad(315), 8, deg2rad(135), 1, 4);
	// range change from 180° with range=3.0 to 135° with range=1.0 exceeds threshold of 1.0
	// new valid minimum is set at 235° with range=2.0
	inputScan.ranges[5] = 2.0;
	filterChain.update(inputScan, filteredScan);
	printScan(filteredScan, f);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{3 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(135);
	shouldScan.angle_max = deg2rad(225);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{inf, 4.f, 2.f};
	shouldScan.intensities = std::vector<float>{inf, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);

	inputScan = generateScan(deg2rad(0), deg2rad(315), 8, deg2rad(270), 1, 2);
	filterChain.update(inputScan, filteredScan);
	printScan(filteredScan, f);

	shouldScan.header.stamp = inputScan.header.stamp + ros::Duration{4 * inputScan.time_increment};
	shouldScan.header.frame_id = inputScan.header.frame_id;
	shouldScan.angle_min = deg2rad(180);
	shouldScan.angle_max = deg2rad(270);
	shouldScan.angle_increment = deg2rad(45);
	shouldScan.time_increment = inputScan.time_increment;
	shouldScan.scan_time = inputScan.scan_time;
	shouldScan.range_min = inputScan.range_min;
	shouldScan.range_max = inputScan.range_max;
	shouldScan.ranges = std::vector<float>{2.f, 2.f, 1.f};
	shouldScan.intensities = std::vector<float>{1.f, 1.f, 1.f};

	compareScans(filteredScan, shouldScan);
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