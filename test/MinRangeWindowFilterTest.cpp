//
// Created by philipp on 27.08.19.
//

#include <gtest/gtest.h>
#include "MinRangeWindowFilter.h"

TEST(MinRangeWindowFilter, fixedMinimum)
{

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
	return RUN_ALL_TESTS();
}