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
