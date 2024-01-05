//
// Created by cc on 11/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_LIMELIGHT_H
#define BOTBUSTERS_REBIRTH_LIMELIGHT_H

#include <bits/stdc++.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include "Core/VisionManager/Sources/VisionSource.h"
#include "LimeLightHelper.h"


class LimeLight {
public:
    LimeLight(std::string name, nt::NetworkTableInstance ntInstance);

    template <typename T>
    T getRobotPose() const;

    std::string getName() const;

    int getBestTagId() const;

    void setLeds(LEDState mode);

    double getLatency() const;

    LEDState getLeds() const;

    bool hasTarget() const;

    [[nodiscard]] units::radian_t getTargetYaw() const;

    [[nodiscard]] units::radian_t  getTargetPitch() const;


    void setPipeLine(int id);

    [[nodiscard]] double getAreaOfImage() const;

    [[nodiscard]] double getTargetingLatency() const;


    [[nodiscard]] int getPipeLine() const;

    LimelightHelpers::VisionResultsClass getResultsFromJSON() const {
        auto out = LimelightHelpers::getLatestResults(fmt::format("limelight-{}", name));
        return out.targetingResults;
    }

    template <typename T>
    T vectorToField(const std::vector<double>& in, double fieldLength, double fieldWidth) const;

    std::vector<AprilTag> getAprilTags() const;



private:

    void updateTelemetry();

    std::shared_ptr<nt::NetworkTable> camera;
    std::shared_ptr<nt::NetworkTable> table;
    std::string name;
    frc::Transform3d camTranslation;
    LEDState state{};

    bool targetValid{};



};


#endif //BOTBUSTERS_REBIRTH_LIMELIGHT_H
