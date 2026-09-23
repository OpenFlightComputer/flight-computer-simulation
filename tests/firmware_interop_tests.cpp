#include "ofcsim/firmware_c.hpp"

#include <cstddef>

#include <gtest/gtest.h>

TEST(FirmwareInterop, DefaultsPassFirmwareValidation)
{
    flight_configuration_t config{};

    flight_configuration_defaults(&config);
    ASSERT_TRUE(flight_configuration_is_valid(&config));
}

TEST(FirmwareInterop, ControlCoreExecutesFromCpp)
{
    const rate_controller_config_t rate_config = {
        .type = RATE_CONTROLLER_TYPE_PID,
        .maximum_gap_us = 10000U,
        .integral_activation_throttle = 0.2F,
        .axis = {
            {.kp = 0.01F, .integral_limit = 0.2F, .output_limit = 1.0F},
            {.kp = 0.01F, .integral_limit = 0.2F, .output_limit = 1.0F},
            {.kp = 0.01F, .integral_limit = 0.2F, .output_limit = 1.0F},
        },
    };
    const roll_attitude_controller_config_t roll = {.gain_per_s = 4.0F};
    const pitch_attitude_controller_config_t pitch = {.gain_per_s = 4.0F};
    const float maximum_rate[RATE_CONTROLLER_AXIS_COUNT] = {
        180.0F, 180.0F, 150.0F,
    };
    prepared_quad_x_mixer_t mixer;
    prepared_control_profile_t profile;
    rate_controller_t rate_controller;
    flight_control_core_t core;
    flight_control_output_t output;
    vehicle_state_t state = {
        .attitude_degrees = {5.0F, -5.0F, 0.0F},
        .angular_rate_dps = {10.0F, -5.0F, 2.0F},
        .acquired_at_us = 1000U,
        .source_sequence = 1U,
        .valid = true,
    };
    control_objective_t objective = {
        .axis_mode = {
            CONTROL_OBJECTIVE_AXIS_ANGLE,
            CONTROL_OBJECTIVE_AXIS_ANGLE,
            CONTROL_OBJECTIVE_AXIS_RATE,
        },
        .axis_value = {12.0F, -6.0F, 75.0F},
        .throttle = 0.5F,
        .produced_at_us = 1000U,
        .valid_until_us = 1000U,
        .valid = true,
    };

    ASSERT_TRUE(quad_x_mixer_prepare(PROPELLER_LAYOUT_PROPS_IN, &mixer));
    ASSERT_TRUE(rate_controller_initialize(&rate_controller, &rate_config));
    ASSERT_TRUE(flight_control_profile_prepare(
        &roll, &pitch, maximum_rate, &mixer, 0.05F, &profile));
    ASSERT_TRUE(flight_control_core_initialize(&core, &rate_controller));

    ASSERT_EQ(flight_control_update(
                  &core, &profile, &state, &objective, 1000U, &output),
              FLIGHT_CONTROL_CORE_UPDATED);
    ASSERT_EQ(output.rate_result, RATE_CONTROLLER_RESULT_SEEDED);
    EXPECT_NEAR(output.desired_rate_dps[0], 28.0F, 0.00001F);
    EXPECT_NEAR(output.desired_rate_dps[1], -4.0F, 0.00001F);
    EXPECT_NEAR(output.desired_rate_dps[2], 75.0F, 0.00001F);
    EXPECT_NEAR(output.motor_baseline, 0.05F, 0.00001F);

    state.acquired_at_us = 2000U;
    state.source_sequence = 2U;
    objective.produced_at_us = 2000U;
    objective.valid_until_us = 2000U;
    objective.axis_mode[RATE_CONTROLLER_AXIS_ROLL] =
        CONTROL_OBJECTIVE_AXIS_RATE;
    objective.axis_mode[RATE_CONTROLLER_AXIS_PITCH] =
        CONTROL_OBJECTIVE_AXIS_RATE;
    objective.axis_value[RATE_CONTROLLER_AXIS_ROLL] = -40.0F;
    objective.axis_value[RATE_CONTROLLER_AXIS_PITCH] = 30.0F;
    ASSERT_EQ(flight_control_update(
                  &core, &profile, &state, &objective, 2000U, &output),
              FLIGHT_CONTROL_CORE_UPDATED);
    ASSERT_EQ(output.rate_result, RATE_CONTROLLER_RESULT_UPDATED);
    EXPECT_NEAR(output.desired_rate_dps[0], -40.0F, 0.00001F);
    EXPECT_NEAR(output.desired_rate_dps[1], 30.0F, 0.00001F);
    EXPECT_NEAR(output.desired_rate_dps[2], 75.0F, 0.00001F);
    ASSERT_TRUE(output.mixer_output_valid);

    objective.throttle = 0.0F;
    objective.produced_at_us = 3000U;
    objective.valid_until_us = 3000U;
    state.valid = false;
    ASSERT_EQ(flight_control_update(
                  &core, &profile, &state, &objective, 3000U, &output),
              FLIGHT_CONTROL_CORE_UPDATED);
    ASSERT_EQ(output.rate_result, RATE_CONTROLLER_RESULT_DISABLED);
    EXPECT_NEAR(output.motor_baseline, 0.05F, 0.00001F);
    for (std::size_t motor = 0U; motor < MOTOR_COMMAND_MOTOR_COUNT; motor++) {
        EXPECT_NEAR(output.mixer_output.command.throttle[motor], 0.05F,
                    0.00001F);
    }

    objective.throttle = 0.5F;
    objective.produced_at_us = 4000U;
    objective.valid_until_us = 4000U;
    ASSERT_EQ(flight_control_update(
                  &core, &profile, &state, &objective, 4000U, &output),
              FLIGHT_CONTROL_CORE_INVALID_INPUT);
    state.valid = true;
    objective.valid_until_us = 3999U;
    ASSERT_EQ(flight_control_update(
                  &core, &profile, &state, &objective, 4000U, &output),
              FLIGHT_CONTROL_CORE_INVALID_INPUT);
}
