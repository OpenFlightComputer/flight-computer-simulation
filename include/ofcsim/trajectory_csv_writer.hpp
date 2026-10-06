#pragma once

#include "ofcsim/plant.hpp"

#include <filesystem>
#include <fstream>

namespace ofcsim {

// Minimal deterministic trajectory output for open-loop experiments.
class TrajectoryCsvWriter {
public:
    explicit TrajectoryCsvWriter(const std::filesystem::path& path);
    ~TrajectoryCsvWriter();

    TrajectoryCsvWriter(const TrajectoryCsvWriter&) = delete;
    TrajectoryCsvWriter& operator=(const TrajectoryCsvWriter&) = delete;

    void write(
        double time_s,
        const PlantState& state,
        const MotorArray& motor_commands);

private:
    std::ofstream output_;
};

}  // namespace ofcsim
