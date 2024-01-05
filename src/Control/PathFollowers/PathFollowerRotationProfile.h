#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <fmt/format.h>
#include <functional>
#include <vector>
#include <cmath>
#include <Core/VisionManager/VisionManager.h>
#include <iostream>

//TODO delet the fuck outta this
struct PathFollowerRotationProfileEntry {
    PathFollowerRotationProfileEntry(double x, double y, double range) {
        this->tX = x;
        this->tY = y;
        this->range = range;
    }

    [[nodiscard]] bool shouldActivate(double x, double y) const {
        double dist = std::hypot(x - tX, y - tY);
//        std::cout << fmt::format("Distance: {}", dist) << std::endl;
        return dist < range;
    }

    virtual frc::Rotation2d getTargetHeading() = 0;

    double tX, tY;
    double range;
};

struct RotationThetaEntry : public PathFollowerRotationProfileEntry {
    RotationThetaEntry(double x, double y, double range, double theta) : PathFollowerRotationProfileEntry(x,
                                                                                                                      y,
                                                                                                                      range) {
        this->theta = theta;
    }


    frc::Rotation2d getTargetHeading() override {
        return {units::radian_t(theta)};

    }

    double theta;
};

struct RotationStationaryEntry : public PathFollowerRotationProfileEntry {
    RotationStationaryEntry(double x, double y, double range, double tX, double tY)
            : PathFollowerRotationProfileEntry(x, y, range) {
        this->targetX = tX;
        this->targetY = tY;
    }

    frc::Rotation2d getTargetHeading() override {
        double dX = targetX - rX;
        double dY = targetY - rY;

        double theta = std::atan2(dY, dX);
//        frc::SmartDashboard::PutNumber("print Theta/ Theta", theta);
        return {units::radian_t(-theta)};
    }

    double rX = 0, rY = 0;
    double targetX = 0, targetY = 0;
};

struct RotationVisionEntry : public PathFollowerRotationProfileEntry {
    RotationVisionEntry(double x, double y, double range,
                                    const std::shared_ptr<EctoSwerve> &swerve,
                                    VisionManager *visionManager)
            : PathFollowerRotationProfileEntry(x, y, range) {
        this->visionManager = visionManager;
        this->swerve = swerve;
    }

    frc::Rotation2d getTargetHeading() override {
        state = 0;
//        frc::SmartDashboard::PutNumber("cameraYawConstant", state);
        swerveAngle = swerve->getPose().Rotation();
        angle = frc::Rotation2d(units::radian_t (state));
        return swerveAngle - angle;
    }

    std::shared_ptr<EctoSwerve> swerve;
    VisionManager * visionManager;

    frc::Rotation2d angle;
    frc::Rotation2d swerveAngle;

    double state = std::numeric_limits<double>::max();
};


class PathFollowerRotationProfile {
public:
    explicit PathFollowerRotationProfile(
            const std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>>& entries, frc::Rotation2d defaultOrientation = {}) {
        this->entries = entries;
        this->lastSetpoint = defaultOrientation;
        this->defaultOrientation = defaultOrientation;
    }

    PathFollowerRotationProfile() {
        ;
    }

    std::function<frc::Rotation2d(double x, double y)> getFun() {
        return [this](const double x, const double y) {
            int i = 0;
            for (const auto &entry: entries) {
                if ((i == 0 && !hasOtherThanFirstTriggered) or entry->shouldActivate(x, y)) {
                    lastSetpoint = entry->getTargetHeading();
//                    std::cout << fmt::format("Setpoint: {}, pose: {}, {}", lastSetpoint.Degrees().value(), x, y) << std::endl;
                    if(i != 0) hasOtherThanFirstTriggered = true;
                }else {
                    lastSetpoint = defaultOrientation;
                }
                i++;
            }
            return lastSetpoint;
        };
    }

private:
    frc::Rotation2d lastSetpoint;
    frc::Rotation2d defaultOrientation;
    std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> entries;
    bool hasOtherThanFirstTriggered{false};
};
