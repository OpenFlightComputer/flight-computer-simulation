#include "ofcsim/imu_model.hpp"

#include <cstdint>
#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

namespace {

constexpr double kGravityMps2 = 9.80665;

ofcsim::ImuModelConfig ideal_imu_config()
{
    ofcsim::ImuModelConfig config{};
    config.acceleration_bias_g.fill(0.0);
    config.gyroscope_bias_dps.fill(0.0);
    config.acceleration_noise_std_g = 0.0;
    config.gyroscope_noise_std_dps = 0.0;
    return config;
}

}  // namespace

TEST(ImuModel, ConvertsSpecificForceAndAngularRateToCounts)
{
    ofcsim::ImuModel model(42U, ideal_imu_config());

    const auto sample = model.sample(
        ofcsim::Vec3(kGravityMps2, -kGravityMps2, 0.0),
        ofcsim::Vec3(
            std::numbers::pi / 180.0,
            -std::numbers::pi / 180.0,
            0.0),
        1000U);

    EXPECT_EQ(sample.acceleration_x, 4096);
    EXPECT_EQ(sample.acceleration_y, -4096);
    EXPECT_EQ(sample.acceleration_z, 0);
    EXPECT_EQ(sample.gyroscope_x, 16);
    EXPECT_EQ(sample.gyroscope_y, -16);
    EXPECT_EQ(sample.gyroscope_z, 0);
}

TEST(ImuModel, LevelRestProducesNegativeOneG)
{
    ofcsim::ImuModel model(42U, ideal_imu_config());

    const auto sample = model.sample(
        ofcsim::Vec3(0.0, 0.0, -kGravityMps2),
        ofcsim::Vec3::Zero(),
        2000U);

    EXPECT_EQ(sample.acceleration_x, 0);
    EXPECT_EQ(sample.acceleration_y, 0);
    EXPECT_EQ(sample.acceleration_z, -4096);
    EXPECT_EQ(sample.gyroscope_x, 0);
    EXPECT_EQ(sample.gyroscope_y, 0);
    EXPECT_EQ(sample.gyroscope_z, 0);
}

TEST(ImuModel, PopulatesSampleMetadataAndIncrementsSequence)
{
    ofcsim::ImuModel model(42U, ideal_imu_config());

    const auto first = model.sample(
        ofcsim::Vec3::Zero(),
        ofcsim::Vec3::Zero(),
        1000U);
    const auto second = model.sample(
        ofcsim::Vec3::Zero(),
        ofcsim::Vec3::Zero(),
        2000U);

    EXPECT_EQ(first.acquired_at_us, 1000U);
    EXPECT_EQ(first.sequence, 1U);
    EXPECT_TRUE(first.valid);
    EXPECT_EQ(second.acquired_at_us, 2000U);
    EXPECT_EQ(second.sequence, 2U);
    EXPECT_TRUE(second.valid);
}

TEST(ImuModel, ResetRestartsSequence)
{
    ofcsim::ImuModel model(42U, ideal_imu_config());

    (void)model.sample(
        ofcsim::Vec3::Zero(),
        ofcsim::Vec3::Zero(),
        1000U);
    model.reset();

    const auto sample = model.sample(
        ofcsim::Vec3::Zero(),
        ofcsim::Vec3::Zero(),
        2000U);

    EXPECT_EQ(sample.sequence, 1U);
}

TEST(ImuModel, ResetCanStartAtSpecifiedSequence)
{
    ofcsim::ImuModel model(42U, ideal_imu_config());
    model.reset(99U);

    const auto sample = model.sample(
        ofcsim::Vec3::Zero(),
        ofcsim::Vec3::Zero(),
        1000U);

    EXPECT_EQ(sample.sequence, 100U);
}

TEST(ImuModel, ExposesConfiguration)
{
    ofcsim::ImuModelConfig config{};
    config.acceleration_counts_per_g = 2048.0;
    config.gyroscope_counts_per_dps = 32.0;

    const ofcsim::ImuModel model(42U, config);

    EXPECT_DOUBLE_EQ(
        model.config().acceleration_counts_per_g,
        2048.0);
    EXPECT_DOUBLE_EQ(
        model.config().gyroscope_counts_per_dps,
        32.0);
}

TEST(ImuModel, DefaultsMatchBlackBoxEstimate)
{
    const ofcsim::ImuModel model(42U);

    EXPECT_NEAR(model.config().acceleration_bias_g.at(0), 0.00643, 1.0e-12);
    EXPECT_NEAR(model.config().acceleration_bias_g.at(1), 0.04479, 1.0e-12);
    EXPECT_NEAR(model.config().acceleration_bias_g.at(2), -0.01316, 1.0e-12);
    EXPECT_NEAR(model.config().gyroscope_bias_dps.at(0), 0.0409, 1.0e-12);
    EXPECT_NEAR(model.config().gyroscope_bias_dps.at(1), 0.1089, 1.0e-12);
    EXPECT_NEAR(model.config().gyroscope_bias_dps.at(2), -0.1413, 1.0e-12);
    EXPECT_DOUBLE_EQ(model.config().acceleration_noise_std_g, 0.0041);
    EXPECT_DOUBLE_EQ(model.config().gyroscope_noise_std_dps, 0.21);
}

TEST(ImuModel, SaturatesCountsToSensorRange)
{
    ofcsim::ImuModel model(42U, ideal_imu_config());

    const auto sample = model.sample(
        ofcsim::Vec3(100.0 * kGravityMps2, 0.0, -100.0 * kGravityMps2),
        ofcsim::Vec3(1000.0, 0.0, -1000.0),
        1000U);

    EXPECT_EQ(sample.acceleration_x, 32767);
    EXPECT_EQ(sample.acceleration_z, -32768);
    EXPECT_EQ(sample.gyroscope_x, 32767);
    EXPECT_EQ(sample.gyroscope_z, -32768);
}

TEST(ImuModel, AppliesConfiguredBiasBeforeQuantization)
{
    ofcsim::ImuModelConfig config{};
    config.acceleration_noise_std_g = 0.0;
    config.gyroscope_noise_std_dps = 0.0;
    config.acceleration_bias_g = {0.1, -0.2, 0.05};
    config.gyroscope_bias_dps = {0.5, -1.0, 2.0};

    ofcsim::ImuModel model(42U, config);
    const auto sample = model.sample(
        ofcsim::Vec3::Zero(),
        ofcsim::Vec3::Zero(),
        1000U);

    EXPECT_EQ(sample.acceleration_x, 410);
    EXPECT_EQ(sample.acceleration_y, -819);
    EXPECT_EQ(sample.acceleration_z, 205);
    EXPECT_EQ(sample.gyroscope_x, 8);
    EXPECT_EQ(sample.gyroscope_y, -16);
    EXPECT_EQ(sample.gyroscope_z, 33);
}

TEST(ImuModel, SameSeedProducesIdenticalGaussianNoise)
{
    ofcsim::ImuModelConfig config{};
    config.acceleration_bias_g.fill(0.0);
    config.gyroscope_bias_dps.fill(0.0);
    config.acceleration_noise_std_g = 0.01;
    config.gyroscope_noise_std_dps = 0.5;

    ofcsim::ImuModel first(123U, config);
    ofcsim::ImuModel second(123U, config);

    for (std::uint64_t sample_index = 0U; sample_index < 10U; ++sample_index) {
        const auto first_sample = first.sample(
            ofcsim::Vec3(0.0, 0.0, -kGravityMps2),
            ofcsim::Vec3::Zero(),
            sample_index * 1000U);
        const auto second_sample = second.sample(
            ofcsim::Vec3(0.0, 0.0, -kGravityMps2),
            ofcsim::Vec3::Zero(),
            sample_index * 1000U);

        EXPECT_EQ(first_sample.acceleration_x, second_sample.acceleration_x);
        EXPECT_EQ(first_sample.acceleration_y, second_sample.acceleration_y);
        EXPECT_EQ(first_sample.acceleration_z, second_sample.acceleration_z);
        EXPECT_EQ(first_sample.gyroscope_x, second_sample.gyroscope_x);
        EXPECT_EQ(first_sample.gyroscope_y, second_sample.gyroscope_y);
        EXPECT_EQ(first_sample.gyroscope_z, second_sample.gyroscope_z);
    }
}

TEST(ImuModel, RejectsNegativeNoiseStandardDeviation)
{
    ofcsim::ImuModelConfig config{};
    config.acceleration_noise_std_g = -0.01;

    EXPECT_THROW(ofcsim::ImuModel(42U, config), std::invalid_argument);
}
