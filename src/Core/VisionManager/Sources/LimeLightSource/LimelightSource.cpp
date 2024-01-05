//
// Created by abiel on 2/25/22.
//

#include "LimelightSource.h"
#include <frc/Timer.h>
#include <iostream>

LimelightSource::LimelightSource(const CameraInfo &info, const std::shared_ptr<EctoSwerve> &swerve) {
    this->swerve = swerve;
    this->info = info;
    camera = std::make_shared<LimeLight>(info.cameraName, ntInstance);
}

void LimelightSource::setLeds(LEDState state) {
    camera->setLeds(state);
}

CameraInfo LimelightSource::getCameraInfo() const {
    return info;
}

TrackedTarget LimelightSource::getTrackedTarget() const {
    TrackedTarget out;
    out.tX = camera->getTargetYaw();
    out.tY = camera->getTargetPitch();
    out.camId = info.globalCameraId;
    out.areaOfImage = camera->getAreaOfImage();
    out.hasTargets = camera->hasTarget();
    out.timestamp = frc::Timer::GetFPGATimestamp() - units::millisecond_t(camera->getLatency());
    out.targetingLatency = units::millisecond_t(camera->getTargetingLatency());
    return out;
}


void LimelightSource::setPipeline(int pipelineId) const{
    camera->setPipeLine(pipelineId);
}

NeuralDetectorResults LimelightSource::getNeuralResults() const {
    return {};
}



AprilTagResults LimelightSource::getAprilTagResults() const {
    AprilTagResults out;
    auto aprilTags = camera->getAprilTags();
    out.tags = aprilTags;
    out.numberOfDetections = aprilTags.size();
    out.cameraInfo = getCameraInfo();
    return out;
}
