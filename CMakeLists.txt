cmake_minimum_required(VERSION 3.25)

project(
    ofcsim
    VERSION 0.1.0
    LANGUAGES C CXX
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(OFCSIM_ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

add_library(ofcsim_warnings INTERFACE)
target_compile_options(ofcsim_warnings INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Werror>
)

add_library(ofcsim_sanitizers INTERFACE)
if(OFCSIM_ENABLE_SANITIZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        message(FATAL_ERROR "The sanitizer preset requires Clang or GCC")
    endif()
    target_compile_options(ofcsim_sanitizers INTERFACE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )
    target_link_options(ofcsim_sanitizers INTERFACE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )
endif()

include(FetchContent)

include(cmake/FirmwareCore.cmake)

add_library(ofcsim STATIC src/ofcsim.cpp)
target_include_directories(ofcsim PUBLIC include)
target_link_libraries(ofcsim
    PUBLIC
        ofc_firmware_core
    PRIVATE
        ofcsim_warnings
        ofcsim_sanitizers
)

add_executable(ofcsim_run apps/ofcsim_run.cpp)
target_link_libraries(ofcsim_run PRIVATE ofcsim ofcsim_warnings ofcsim_sanitizers)

include(CTest)
if(BUILD_TESTING)
    find_package(GTest 1.15.2 QUIET)
    if(NOT GTest_FOUND)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
            URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
        )
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()
    add_subdirectory(tests)
endif()
