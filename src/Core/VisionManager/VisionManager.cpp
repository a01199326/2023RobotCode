//
// Created by abiel on 2/12/20.
//

#include "VisionManager.h"
#include "frc/StateSpaceUtil.h"
#include <Systems/EctoSwerve/EctoSwerve.h>
#include "Math/VisionUtilities.h"
#include <cmath>
#include "Utilities/ChecklistTests/SSHConnectionTest.h"

VisionManager::VisionManager(const std::shared_ptr<EctoSwerve> &swerve,
                             const std::vector<std::shared_ptr<VisionSource>> &sources,
                             VisionManagerConfig &config) :
                             WPISubsystem("VisionManager"){
    this->config = config;
    this->swerve = swerve;
    this->tagPoses = config.tagPoses;
    this->sources = sources;

    visionStdDevs = std::make_shared<InterpolatingTable>();
    visionStdDevs->addPointList(config.visionStdDevs);


    visionNotifier = std::make_unique<frc::Notifier>([&] {
//        std::vector<AprilTagResults> tagResults;
//        for (auto &source: sources) {
//            auto tagResult = source->getAprilTagResults();
//            if (EctoMath::epsilonEquals(tagResult.numberOfDetections, -1)) continue;
//            tagResults.emplace_back(tagResult);
//        }
//
////        if (!tagResults.empty()) updateTagVision(tagResults);
//
//        lastVisionRunTime = frc::Timer::GetFPGATimestamp();
    });

#ifndef SIMULATION
    visionNotifier->StartPeriodic(units::millisecond_t(20));
#endif
}

void VisionManager::autoInit() {

}

void VisionManager::teleopInit() {

}

double VisionManager::getTargetDistance() const {
    return swerve->getPose().Translation().Distance(config.fieldToTarget.Translation()).value();
}


void VisionManager::ignoreVision(bool ignoreVision) {
    useVision = !ignoreVision;
}


void VisionManager::updateTagVision(const std::vector<AprilTagResults> &results) {

//    for (auto &result : results){
//        if (!result.hasTargets){
//            continue;
//        }
//        if (result.numberOfDetections > 1){
//            swerve->setVisionMeasurementStdDevs(wpi::array<double, 3>{0.025, 0.025, 0.025});
//            bool ok = VisionUtilities::filterTagPose(result.bestTag, config, swerve, log);
//            if (ok && useVision) swerve->addVisionPoseMeasurement({result.bestTag.r_fs.ToPose2d().Translation(), swerve->getPose().Rotation()}, result.timestamp.value());
//        }
//        for (auto &tag : result.tags){
//            if (tag.latency > 150_ms){
//                std::cout << fmt::format("latency to high in tag detection: {}s", tag.latency) << std::endl;
//                continue;
//            };
//            if (tag.id == -1){
//                std::cout << "invalid tag id" << std::endl;
//                continue;
//            }
//
//            auto speeds = swerve->getChassisSpeeds();
//            double angularVel = speeds.omega();
//            double velMag = frc::Velocity2d(speeds.vx, speeds.vy).Norm().value();
//            double fastestSpeed = std::max(velMag, angularVel);
//
//            double stdOut = EctoMath::map(fastestSpeed, 0.0, 6.0, minVisionStdDev, maxVisionStdDev);
//            double visionStd = visionStdDevs->get(tag.areaOfImage);
//            stdOut += visionStd;
//            swerve->setVisionMeasurementStdDevs(wpi::array<double, 3>{stdOut, stdOut, stdOut});
//
//            bool filterTagPoses = VisionUtilities::filterTagPose(tag, config, swerve, log);
//            if (filterTagPoses && useVision) swerve->addVisionPoseMeasurement({tag.r_fs.ToPose2d().Translation(), swerve->getPose().Rotation()}, result.timestamp.value());
//        }
//    }

}

void VisionManager::robotUpdate() {

    auto dt = frc::Timer::GetFPGATimestamp() - lastTranslationToTargetTime;
    frc::Pose2d poseRelative = swerve->getPose().RelativeTo(config.fieldToTarget);
    frc::Twist2d twistToTarget;
    twistToTarget.dx = poseRelative.X();
    twistToTarget.dy = poseRelative.Y();
    twistToTarget.dtheta = poseRelative.Rotation().Radians();

    twistDelta = twistToTarget;
    twistDelta.dx -= lastTwistToTarget.dx;
    twistDelta.dy -= lastTwistToTarget.dy;
    twistDelta.dtheta -= lastTwistToTarget.dtheta;
    twistDelta = twistDelta * (1.0/dt.value());

    lastTwistToTarget = twistToTarget;
    lastTranslationToTargetTime = frc::Timer::GetFPGATimestamp();

//    table->GetEntry("TargetPos/X").SetDouble(twistDelta.dx.value());
//    table->GetEntry("TargetPos/Y").SetDouble(twistDelta.dy.value());
//    table->GetEntry("ProcessedFrames").SetDouble(processedFrames);
//    table->GetEntry("DroppedFrames").SetDouble(droppedFrames);

}

frc::Pose2d VisionManager::getVisionPose() const {
    return visionPose;
}


void VisionManager::updateVisionTelemetry(const std::vector<AprilTagResults> &results, const CameraInfo &cameraInfo) {
    return;
    if (results.empty()){
        return;
    }
//    for (auto &result : results){
//        if (!result.hasTargets){
//            continue;
//        }
//
////        for (auto &tag : result.tags){
////            if (tag.latency > 1.5_s){
////                std::cout << "too much latency" << std::endl;
////                continue;
////            };
////            if (tag.id == -1){
////                std::cout << "invalid tag id" << std::endl;
////                continue;
////            }
////
////            VisionUtilities::publishTag(tag, tagsTable);
////        }
//
//    }
}

frc::Twist2d VisionManager::getVelocityToTarget() const {
    return twistDelta;
}

void VisionManager::setLeds(bool state, int cameraIndex) {
    sources[cameraIndex]->setLeds(state ? LEDState::kON : LEDState::kOff);
}

std::optional<units::meter_t> VisionManager::getYOffsetToTarget(int cameraId) const{
    if (!sources[cameraId]->getTrackedTarget().hasTargets){
        return {};
    }
    auto target = sources[cameraId]->getTrackedTarget();
    return VisionUtilities::calculateYOffset(target, swerve, config, cameraId);

}

std::optional<units::meter_t> VisionManager::getXOffsetToTarget(int cameraId) const {
    if (!sources[cameraId]->getTrackedTarget().hasTargets) {
        return {};
    }
    auto target = sources[cameraId]->getTrackedTarget();
    return VisionUtilities::calculateXOffset(target, config, cameraId);
}

void VisionManager::setPipeline(int cameraId, int pipelineId) {
    sources[cameraId]->setPipeline(pipelineId);
}

std::vector<std::shared_ptr<ChecklistItem>> VisionManager::createTests() {
    return {
            std::make_shared<SSHConnectionTest>("10.46.35.6")
    };
}


