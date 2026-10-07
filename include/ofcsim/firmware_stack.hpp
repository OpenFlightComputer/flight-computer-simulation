#pragma once

#include "ofcsim/firmware_c.hpp"

#include <cstdint>

namespace ofcsim {

class FirmwareStack {
public:
    explicit FirmwareStack(const flight_configuration_t& configuration);

    FirmwareStack(const FirmwareStack&) = delete;
    FirmwareStack& operator=(const FirmwareStack&) = delete;
    FirmwareStack(FirmwareStack&&) = delete;
    FirmwareStack& operator=(FirmwareStack&&) = delete;

    void process_imu(
        const imu_sample_snapshot_t& sample,
        imu_freshness_t freshness);

    [[nodiscard]] vehicle_state_t vehicle_state() const;

    [[nodiscard]] flight_control_core_result_t run_core(
        const control_objective_t& objective,
        const vehicle_state_t& state,
        std::uint64_t now_us,
        flight_control_output_t& output);

    void reset_core();

    [[nodiscard]] const flight_configuration_t& configuration() const;
    [[nodiscard]] const prepared_control_input_shaping_t& prepared_control() const;
    [[nodiscard]] const imu_processing_pipeline_t& imu_pipeline() const;

private:
    flight_configuration_t config_{};
    prepared_control_input_shaping_t control_{};
    prepared_quad_x_mixer_t mixer_{};
    prepared_control_profile_t profile_{};
    rate_controller_t rate_controller_{};
    flight_control_core_t core_{};
    imu_processing_pipeline_t imu_{};
};

}  // namespace ofcsim
