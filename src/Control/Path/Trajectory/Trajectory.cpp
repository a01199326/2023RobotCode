//
// Created by cc on 7/12/23.
//

#include "Trajectory.h"

#include <utility>
using namespace botbusters;

#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/json.h>
#include <fstream>
#include <wpi/raw_istream.h>
#include <iostream>

std::vector<TrajectoryState> Trajectory::readJSON(const std::string &pathName) {
    wpi::json root;
    fs::path deployDir = frc::filesystem::GetDeployDirectory();
    deployDir = deployDir / "Paths" / pathName;
//    deployDir = "/home/cc/Botbusters_Rebirth/deploy/Paths/" + pathName;
    std::error_code ec;
    wpi::raw_fd_istream input{deployDir.string(), ec};
    std::cout << deployDir.string()  << std::endl;
    input >> root;

    auto samples = root["samples"];

    std::vector<TrajectoryState> states;
    for (auto& sample : samples) {
        states.emplace_back(
                units::second_t(
                        sample["timestamp"].get<double>()
                ),
                frc::Pose2d(units::meter_t(sample["x"].get<double>()),
                            units::meter_t(sample["y"].get<double>()),
                            {units::radian_t(sample["heading"].get<double>())}),
                frc::Velocity2d(
                        units::meters_per_second_t(sample["velocityX"].get<double>()),
                        units::meters_per_second_t(sample["velocityY"].get<double>())
                ),
                units::radians_per_second_t(
                        sample["angularVelocity"].get<double>()
                ),
                frc::Acceleration2d(
                        0_mps_sq,
                        0_mps_sq
                )
        );
    }

    return {std::move(states)};
}



