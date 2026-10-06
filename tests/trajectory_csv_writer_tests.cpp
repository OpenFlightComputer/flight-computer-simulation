#include "ofcsim/trajectory_csv_writer.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace {

TEST(TrajectoryCsvWriter, WritesHeaderAndStateRow)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        "ofcsim_trajectory_csv_writer_test.csv";
    std::filesystem::remove(path);

    ofcsim::PlantState state{};
    state.rigid_body.position_m = ofcsim::Vec3(1.0, 2.0, -3.0);
    state.rigid_body.velocity_mps = ofcsim::Vec3(4.0, 5.0, 6.0);
    state.rotor_speed_rad_s = {7.0, 8.0, 9.0, 10.0};
    const ofcsim::MotorArray commands{0.1, 0.2, 0.3, 0.4};

    {
        ofcsim::TrajectoryCsvWriter writer(path);
        writer.write(0.125, state, commands);
    }

    std::ifstream input(path);
    ASSERT_TRUE(input);
    std::string header;
    std::string row;
    ASSERT_TRUE(std::getline(input, header));
    ASSERT_TRUE(std::getline(input, row));

    EXPECT_EQ(
        header,
        "time_s,position_x_m,position_y_m,position_z_m,"
        "velocity_x_mps,velocity_y_mps,velocity_z_mps,"
        "rotor_speed_0_rad_s,rotor_speed_1_rad_s,"
        "rotor_speed_2_rad_s,rotor_speed_3_rad_s,"
        "motor_command_0,motor_command_1,motor_command_2,motor_command_3");

    std::stringstream values(row);
    std::string value;
    int count = 0;
    while (std::getline(values, value, ',')) {
        ++count;
    }
    EXPECT_EQ(count, 15);

    std::filesystem::remove(path);
}

TEST(TrajectoryCsvWriter, OverwritesExistingFile)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        "ofcsim_trajectory_csv_writer_overwrite_test.csv";
    {
        std::ofstream old_file(path);
        old_file << "old content\n";
    }

    {
        ofcsim::TrajectoryCsvWriter writer(path);
        writer.write(0.0, ofcsim::PlantState{}, ofcsim::MotorArray{});
    }

    std::ifstream input(path);
    ASSERT_TRUE(input);
    std::string first_line;
    ASSERT_TRUE(std::getline(input, first_line));
    EXPECT_NE(first_line.find("time_s"), std::string::npos);
    EXPECT_EQ(first_line.find("old content"), std::string::npos);

    std::filesystem::remove(path);
}

}  // namespace
