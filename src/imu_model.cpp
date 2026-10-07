#include "ofcsim/imu_model.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace ofcsim {

ImuModel::ImuModel(
    std::uint64_t seed,
    ImuModelConfig config)
    : config_(config),
      seed_(seed),
      random_engine_(seed)
{
    // TODO(M4): Move IMU calibration and noise fields into VehicleConfig and
    // validate them in the vehicle configuration loader.
    if (!std::isfinite(config_.acceleration_counts_per_g) ||
        config_.acceleration_counts_per_g <= 0.0 ||
        !std::isfinite(config_.gyroscope_counts_per_dps) ||
        config_.gyroscope_counts_per_dps <= 0.0 ||
        !std::isfinite(config_.acceleration_noise_std_g) ||
        config_.acceleration_noise_std_g < 0.0 ||
        !std::isfinite(config_.gyroscope_noise_std_dps) ||
        config_.gyroscope_noise_std_dps < 0.0) {
        throw std::invalid_argument("invalid IMU scale or noise configuration");
    }

    for (const double bias : config_.acceleration_bias_g) {
        if (!std::isfinite(bias)) {
            throw std::invalid_argument("invalid accelerometer bias");
        }
    }
    for (const double bias : config_.gyroscope_bias_dps) {
        if (!std::isfinite(bias)) {
            throw std::invalid_argument("invalid gyroscope bias");
        }
    }
}

imu_sample_snapshot_t ImuModel::sample(
    const Vec3& specific_force_body_mps2,
    const Vec3& rate_body_rad_s,
    std::uint64_t time_us)
{
    imu_sample_snapshot_t sample{};

    sample.acceleration_x =
        convert_specific_force(specific_force_body_mps2.x(), 0U);
    sample.acceleration_y =
        convert_specific_force(specific_force_body_mps2.y(), 1U);
    sample.acceleration_z =
        convert_specific_force(specific_force_body_mps2.z(), 2U);

    sample.gyroscope_x = convert_angular_rate(rate_body_rad_s.x(), 0U);
    sample.gyroscope_y = convert_angular_rate(rate_body_rad_s.y(), 1U);
    sample.gyroscope_z = convert_angular_rate(rate_body_rad_s.z(), 2U);

    sample.acquired_at_us = time_us;
    sample.sequence = ++sequence_;
    sample.valid = true;

    return sample;
}

void ImuModel::reset(std::uint64_t sequence)
{
    random_engine_.seed(seed_);
    sequence_ = sequence;
}

const ImuModelConfig& ImuModel::config() const
{
    return config_;
}

std::int32_t ImuModel::to_counts(double value)
{
    const long rounded = std::lround(value);
    return static_cast<std::int32_t>(
        std::clamp(rounded, -32768L, 32767L));
}

std::int32_t ImuModel::convert_specific_force(
    double specific_force,
    std::size_t axis)
{
    constexpr double kStandardGravity = 9.80665;
    const double acceleration_g =
        specific_force / kStandardGravity +
        config_.acceleration_bias_g.at(axis) +
        sample_noise(config_.acceleration_noise_std_g);

    return to_counts(acceleration_g * config_.acceleration_counts_per_g);
}

std::int32_t ImuModel::convert_angular_rate(
    double rate_rad_s,
    std::size_t axis)
{
    constexpr double kRadToDeg = 180.0 / std::numbers::pi;
    const double rate_dps =
        rate_rad_s * kRadToDeg +
        config_.gyroscope_bias_dps.at(axis) +
        sample_noise(config_.gyroscope_noise_std_dps);

    return to_counts(rate_dps * config_.gyroscope_counts_per_dps);
}

double ImuModel::sample_noise(double standard_deviation)
{
    if (standard_deviation == 0.0) {
        return 0.0;
    }

    std::normal_distribution<double> distribution(0.0, standard_deviation);
    return distribution(random_engine_);
}

}  // namespace ofcsim
