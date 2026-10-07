#include "ofcsim/firmware_stack.hpp"

namespace ofcsim {

FirmwareStack::FirmwareStack(const flight_configuration_t& configuration)
    : config_(configuration)
{
    (void)control_;
    (void)mixer_;
    (void)profile_;
    (void)rate_controller_;
    (void)core_;

    // TODO(M5): Mirror the firmware runtime preparation and initialize every
    // owned subsystem from the pinned firmware implementation.
}

void FirmwareStack::process_imu(
    const imu_sample_snapshot_t& sample,
    imu_freshness_t freshness)
{
    (void)sample;
    (void)freshness;

    // TODO(M5): Pass the sensor sample to the real firmware IMU pipeline.
}

vehicle_state_t FirmwareStack::vehicle_state() const
{
    // TODO(M5): Read the firmware attitude snapshot and build vehicle_state_t
    // using the same mapping as the flight-control task.
    return {};
}

flight_control_core_result_t FirmwareStack::run_core(
    const control_objective_t& objective,
    const vehicle_state_t& state,
    std::uint64_t now_us,
    flight_control_output_t& output)
{
    (void)objective;
    (void)state;
    (void)now_us;
    output = {};

    // TODO(M5): Call the real firmware control core and return its result.
    return FLIGHT_CONTROL_CORE_NOT_INITIALIZED;
}

void FirmwareStack::reset_core()
{
    // TODO(M5): Reset the real firmware control core state.
}

const flight_configuration_t& FirmwareStack::configuration() const
{
    // TODO(M5): Return the validated firmware configuration.
    return config_;
}

const prepared_control_input_shaping_t& FirmwareStack::prepared_control() const
{
    // TODO(M5): Return the firmware-prepared input-shaping configuration.
    return control_;
}

const imu_processing_pipeline_t& FirmwareStack::imu_pipeline() const
{
    // TODO(M5): Return the firmware IMU pipeline for diagnostics and logging.
    return imu_;
}

}  // namespace ofcsim
