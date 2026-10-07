#pragma once

#include "ofcsim/firmware_c.hpp"
#include "ofcsim/rigid_body.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>

namespace ofcsim {

struct ImuModelConfig {
    double acceleration_counts_per_g = 4096.0;
    double gyroscope_counts_per_dps = 16.384;
    // Initial estimates from stationary BMI270 black-box samples.
    std::array<double, 3> acceleration_bias_g = {
        0.00643, 0.04479, -0.01316};
    std::array<double, 3> gyroscope_bias_dps = {
        0.0409, 0.1089, -0.1413};
    double acceleration_noise_std_g = 0.0041;
    double gyroscope_noise_std_dps = 0.21;
};

class ImuModel {
public:
    explicit ImuModel(
        std::uint64_t seed,
        ImuModelConfig config = {});

    [[nodiscard]] imu_sample_snapshot_t sample(
        const Vec3& specific_force_body_mps2,
        const Vec3& rate_body_rad_s,
        std::uint64_t time_us);

    void reset(std::uint64_t sequence = 0U);

    [[nodiscard]] const ImuModelConfig& config() const;

private:
    [[nodiscard]] std::int32_t convert_specific_force(
        double specific_force,
        std::size_t axis);

    [[nodiscard]] std::int32_t convert_angular_rate(
        double rate_rad_s,
        std::size_t axis);

    [[nodiscard]] double sample_noise(double standard_deviation);

    [[nodiscard]] static std::int32_t to_counts(double value);

    ImuModelConfig config_{};
    std::uint64_t seed_ = 0U;
    std::mt19937_64 random_engine_;
    std::uint64_t sequence_ = 0U;
};



}  // namespace ofcsim
