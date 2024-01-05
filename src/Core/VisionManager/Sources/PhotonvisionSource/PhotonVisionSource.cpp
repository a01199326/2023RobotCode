//
// Created by abiel on 2/25/22.
//

#include "PhotonVisionSource.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

PhotonVisionSource::PhotonVisionSource(const CameraInfo &info, const std::shared_ptr<EctoSwerve> &swerve) {
    this->swerve = swerve;
    this->info = info;
    camera = std::make_shared<photonlib::PhotonCamera>("camera");

}

void PhotonVisionSource::setLeds(LEDState state) {
    switch (state) {
        case LEDState::kON:
            camera->SetLEDMode(photonlib::LEDMode::kOn);
            break;
        case LEDState::kOff:
            camera->SetLEDMode(photonlib::LEDMode::kOff);
            break;
        case LEDState::kBlink:
            camera->SetLEDMode(photonlib::LEDMode::kBlink);
            break;
    }
}

CameraInfo PhotonVisionSource::getCameraInfo() const {
    return info;
}

void PhotonVisionSource::setPipeline(int pipelineId) const {
    camera->SetPipelineIndex(pipelineId);
}

TrackedTarget PhotonVisionSource::getTrackedTarget() const {
    TrackedTarget out;
    auto bestResult = camera->GetLatestResult().GetBestTarget();
    out.tX = units::radian_t(bestResult.GetPitch());
    out.tY = units::radian_t(bestResult.GetYaw());
    out.camId = info.globalCameraId;
    out.areaOfImage = bestResult.GetArea() / 100;
    out.hasTargets = true;
    out.timestamp = frc::Timer::GetFPGATimestamp() - camera->GetLatestResult().GetLatency();
    out.targetingLatency = units::millisecond_t(0);
    return out;
}

NeuralDetectorResults PhotonVisionSource::getNeuralResults() const {
    return {};
}

AprilTagResults PhotonVisionSource::getAprilTagResults() const{
    AprilTagResults out;
    auto latestResults = camera->GetLatestResult();
    auto dump = latestResults.GetTargets();
    std::vector<AprilTag> tags;
    for (auto &result : dump) {
        AprilTag tag;
        units::second_t latency = latestResults.GetLatency();
        tag.c_ts = result.GetBestCameraToTarget();// PhotonVision can only return cameraPose in target space with the best camera to target
        result.GetBestCameraToTarget();

        tags.emplace_back(tag);
    }
    out.numberOfDetections = dump.size();
    out.cameraInfo = getCameraInfo();
    return out;
}
