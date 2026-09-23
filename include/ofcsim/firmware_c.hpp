#pragma once

// This is the only simulator header that includes firmware C headers.
extern "C" {
#include "bmi270_configuration.h"
#include "control_input_shaping.h"
#include "flight_configuration.h"
#include "flight_configuration_snapshot.h"
#include "flight_control_core.h"
#include "flight_runtime_configuration.h"
#include "imu_processing_pipeline.h"
#include "imu_sample.h"
#include "manual_easy_behavior.h"
#include "receiver_failsafe.h"
#include "vehicle_state.h"
}
