#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "signal_processing/filters.hpp"

TEST(LowPassFilter, StartsAtFirstSampleAndConverges)
{
  signal_processing::LowPassFilter filter(10.0, 100.0);

  EXPECT_DOUBLE_EQ(filter.update(0.0), 0.0);
  double output = 0.0;
  for (int index = 0; index < 100; ++index) {
    output = filter.update(1.0);
  }

  EXPECT_NEAR(output, 1.0, 1.0e-6);
  EXPECT_GT(filter.alpha(), 0.0);
  EXPECT_LT(filter.alpha(), 1.0);
}

TEST(LowPassFilter, UsesExpectedDefaultCoefficient)
{
  signal_processing::LowPassFilter filter(60.0, 1000.0);

  EXPECT_NEAR(filter.alpha(), 0.314077834, 1.0e-9);
}

TEST(LowPassFilter, ResetUsesTheNextSampleAsAStartingPoint)
{
  signal_processing::LowPassFilter filter(10.0, 100.0);
  filter.update(0.0);
  filter.update(100.0);
  filter.reset();

  EXPECT_DOUBLE_EQ(filter.update(-3.0), -3.0);
}

TEST(LowPassFilter, RejectsInvalidConfigurationAndInput)
{
  EXPECT_THROW(signal_processing::LowPassFilter(0.0, 100.0), std::invalid_argument);
  EXPECT_THROW(signal_processing::LowPassFilter(50.0, 100.0), std::invalid_argument);

  signal_processing::LowPassFilter filter(10.0, 100.0);
  EXPECT_THROW(
    filter.update(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(MedianFilter, RemovesAnImpulseOutlier)
{
  signal_processing::MedianFilter filter(5);

  EXPECT_DOUBLE_EQ(filter.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(filter.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(filter.update(100.0), 1.0);
  EXPECT_DOUBLE_EQ(filter.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(filter.update(1.0), 1.0);
  EXPECT_EQ(filter.sample_count(), 5U);
}

TEST(MedianFilter, KeepsOnlyTheConfiguredWindow)
{
  signal_processing::MedianFilter filter(3);
  filter.update(1.0);
  filter.update(2.0);
  filter.update(3.0);

  EXPECT_DOUBLE_EQ(filter.update(100.0), 3.0);
  EXPECT_EQ(filter.sample_count(), 3U);
}

TEST(MedianFilter, ResetClearsTheWindow)
{
  signal_processing::MedianFilter filter(3);
  filter.update(1.0);
  filter.update(100.0);
  filter.reset();

  EXPECT_EQ(filter.sample_count(), 0U);
  EXPECT_DOUBLE_EQ(filter.update(-2.0), -2.0);
}

TEST(MedianFilter, RejectsInvalidWindowAndInput)
{
  EXPECT_THROW(signal_processing::MedianFilter(1), std::invalid_argument);
  EXPECT_THROW(signal_processing::MedianFilter(4), std::invalid_argument);

  signal_processing::MedianFilter filter(3);
  EXPECT_THROW(
    filter.update(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
