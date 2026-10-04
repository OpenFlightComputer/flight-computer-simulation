add_executable(firmware_interop_tests firmware_interop_tests.cpp)
target_link_libraries(firmware_interop_tests
    PRIVATE
        ofcsim
        GTest::gtest_main
        ofcsim_warnings
        ofcsim_sanitizers
)

include(GoogleTest)
gtest_discover_tests(firmware_interop_tests)

add_executable(rigid_body_tests rigid_body_tests.cpp)
target_link_libraries(rigid_body_tests
    PRIVATE
        ofcsim
        GTest::gtest_main
        ofcsim_warnings
        ofcsim_sanitizers
)

include(GoogleTest)
gtest_discover_tests(rigid_body_tests)

add_executable(vehicle_config_tests vehicle_config_tests.cpp)
target_link_libraries(vehicle_config_tests
    PRIVATE
        ofcsim
        GTest::gtest_main
        ofcsim_warnings
        ofcsim_sanitizers
)
target_compile_definitions(vehicle_config_tests
    PRIVATE
        OFCSIM_TEST_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/resources"
)
configure_file(
    resources/ofc_vehicle_config_test.json
    ${CMAKE_CURRENT_BINARY_DIR}/resources/ofc_vehicle_config_test.json
    COPYONLY
)
configure_file(
    resources/malformed_vehicle_config.json
    ${CMAKE_CURRENT_BINARY_DIR}/resources/malformed_vehicle_config.json
    COPYONLY
)

gtest_discover_tests(vehicle_config_tests)

add_executable(propulsion_tests propulsion_tests.cpp)
target_link_libraries(propulsion_tests
    PRIVATE
        ofcsim
        GTest::gtest_main
        ofcsim_warnings
        ofcsim_sanitizers
)

gtest_discover_tests(propulsion_tests)

add_executable(plant_tests plant_tests.cpp)
target_link_libraries(plant_tests
    PRIVATE
        ofcsim
        GTest::gtest_main
        ofcsim_warnings
        ofcsim_sanitizers
)

gtest_discover_tests(plant_tests)
