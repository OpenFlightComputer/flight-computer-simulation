set(OFC_FIRMWARE_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/external/flight-computer-firmware")
set(OFC_FIRMWARE_SOURCE "${OFC_FIRMWARE_ROOT}/firmware")

if(NOT EXISTS "${OFC_FIRMWARE_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR
        "Firmware submodule is missing. Run: git submodule update --init --recursive")
endif()

set(OFC_DEFAULT_CONFIGURATION_FILE
    "${OFC_FIRMWARE_ROOT}/config/default-flight-configuration.json")
include("${OFC_FIRMWARE_ROOT}/cmake/FlightConfiguration.cmake")

add_library(ofc_firmware_core STATIC
    # Canonical control boundary.
    "${OFC_FIRMWARE_SOURCE}/flight/control/flight_control_core.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/quad_x_mixer.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/roll_attitude_controller.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/pitch_attitude_controller.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/yaw_attitude_controller.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/rate_pid.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/rate_controller.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/control_curve.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/control_input_shaping.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/takeoff_leveling.c"
    "${OFC_FIRMWARE_SOURCE}/flight/control/vehicle_state.c"

    # Motor command data model.
    "${OFC_FIRMWARE_SOURCE}/flight/actuators/motor_command.c"
    "${OFC_FIRMWARE_SOURCE}/flight/actuators/motor_configuration.c"

    # Manual behavior and receiver failsafe.
    "${OFC_FIRMWARE_SOURCE}/flight/behavior/common/flight_behavior_configuration.c"
    "${OFC_FIRMWARE_SOURCE}/flight/behavior/manual_easy/manual_easy_behavior.c"
    "${OFC_FIRMWARE_SOURCE}/flight/receiver/receiver_failsafe.c"

    # IMU processing and estimation.
    "${OFC_FIRMWARE_SOURCE}/flight/sensors/imu_sample.c"
    "${OFC_FIRMWARE_SOURCE}/flight/sensors/acceleration_filter.c"
    "${OFC_FIRMWARE_SOURCE}/flight/sensors/gyro_filter.c"
    "${OFC_FIRMWARE_SOURCE}/flight/sensors/level_calibration.c"
    "${OFC_FIRMWARE_SOURCE}/flight/estimation/accelerometer_attitude.c"
    "${OFC_FIRMWARE_SOURCE}/flight/estimation/attitude_estimator.c"
    "${OFC_FIRMWARE_SOURCE}/flight/estimation/imu_processing_pipeline.c"

    # Defaults, runtime preparation, and log-snapshot replay.
    "${OFC_FIRMWARE_SOURCE}/flight/configuration/flight_configuration.c"
    "${OFC_FIRMWARE_SOURCE}/flight/configuration/flight_runtime_configuration.c"
    "${OFC_FIRMWARE_SOURCE}/flight/configuration/flight_configuration_snapshot.c"
)

set_target_properties(ofc_firmware_core PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

target_include_directories(ofc_firmware_core PUBLIC
    "${OFC_FIRMWARE_SOURCE}/flight/actuators"
    "${OFC_FIRMWARE_SOURCE}/flight/behavior/common"
    "${OFC_FIRMWARE_SOURCE}/flight/behavior/manual_easy"
    "${OFC_FIRMWARE_SOURCE}/flight/configuration"
    "${OFC_FIRMWARE_SOURCE}/flight/control"
    "${OFC_FIRMWARE_SOURCE}/flight/estimation"
    "${OFC_FIRMWARE_SOURCE}/flight/receiver"
    "${OFC_FIRMWARE_SOURCE}/flight/sensors"
    "${OFC_FIRMWARE_SOURCE}/peripherals/bmi270"
    "${CMAKE_BINARY_DIR}/generated"
)

target_compile_options(ofc_firmware_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
target_link_libraries(ofc_firmware_core
    PUBLIC m
    PRIVATE ofcsim_sanitizers
)
