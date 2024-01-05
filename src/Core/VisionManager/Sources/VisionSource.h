//
// Created by abiel on 2/25/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONSOURCE_H
#define BOTBUSTERS_REBIRTH_VISIONSOURCE_H

#include <vector>
#include <units/time.h>
#include <units/angle.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

enum class LEDState{
    kON,
    kOff,
    kBlink
};

enum class Software {
    kLIMELIGHT,
    kPHOTONVISION,
    kSLIMELIGHT
};

struct CameraInfo {
    int globalCameraId;
    std::pair<double,double> resolution;
    units::degree_t verticalFov, horizontalFov;
    Software software;
    frc::Pose3d cameraPose;
    LEDState ledState;
    std::string cameraName;

};

struct TrackedTarget{
    units::radian_t tX{};
    units::radian_t tY{};
    bool hasTargets{false};
    int camId{-1};
    units::second_t timestamp;
    units::second_t targetingLatency;
    double areaOfImage;
};

struct AprilTag{
    frc::Transform3d c_ts{};// Camera Pose Target Space
    frc::Transform3d r_fs{};// Robot pose field Space
    frc::Transform3d r_ts{};// Robot Pose target space
    frc::Transform3d t_cs{};// target Pose camera space
    frc::Transform3d t_rs{};// target pose Robot Space
    units::radian_t robotYaw{};
    int tagId{-1};
    TrackedTarget targetInfo{};
};

struct NeuralResults {
    std::string type;
    TrackedTarget targetInfo{};
};

struct AprilTagResults{
    std::vector<AprilTag> tags;
    int numberOfDetections = -1;
    CameraInfo cameraInfo;

};

struct NeuralDetectorResults {
    std::vector<NeuralResults> results;
    CameraInfo cameraInfo;
};





class VisionSource {
public:
    virtual void setLeds(LEDState state) = 0;

    [[nodiscard]] virtual CameraInfo getCameraInfo() const = 0;

    virtual TrackedTarget getTrackedTarget() const = 0;

    virtual NeuralDetectorResults getNeuralResults() const = 0;

    virtual AprilTagResults getAprilTagResults() const = 0;

    virtual void setPipeline(int pipelineId) const = 0;
};

#endif //BOTBUSTERS_REBIRTH_VISIONSOURCE_H
