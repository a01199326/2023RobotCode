//
// Created by cc on 11/01/23.
//

#include "LimeLight.h"
#include <wpi/json.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Math/VisionUtilities.h"

LimeLight::LimeLight(std::string name, nt::NetworkTableInstance ntInstance) {
    this->name = name;
    camera = ntInstance.GetTable(fmt::format("limelight-{}", name));
    table = ntInstance.GetTable(fmt::format("Limelight/Internal/{}", name));
}

template <typename T>
T LimeLight::getRobotPose() const {
    auto camPose = camera->GetEntry("botpose").GetDoubleArray({});
    if (camPose.empty()){
        return {};
    }

//    table->GetEntry("vectorOut").SetDoubleArray(camPose);
    return vectorToField<T>(camPose, 16.459, 8.23);
}

template <typename T>
T LimeLight::vectorToField(const std::vector<double> &in, double fieldLength, double fieldWidth) const {
    auto x = units::meter_t((fieldLength / 2) + in[0]);
    auto y = units::meter_t((fieldWidth / 2) + in[1]);
    auto z = units::meter_t(in[2]);
    auto translation = frc::Translation3d(x, y, z);
    auto rx = units::degree_t((in[3]));
    auto ry = units::degree_t((in[4]));
    auto rz = units::degree_t((in[5]));
    auto rotation = frc::Rotation3d(rx, ry, rz);

//    table->GetEntry("vectorToField/x").SetDouble(x.value());
//    table->GetEntry("vectorToField/y").SetDouble(y.value());
//    table->GetEntry("vectorToField/z").SetDouble(z.value());
//    table->GetEntry("vectorToField/yaw").SetDouble(EctoMath::radiansToDegrees(rotation.Z().value()));
//    table->GetEntry("vectorToField/pitch").SetDouble(EctoMath::radiansToDegrees(rotation.Y().value()));
//    table->GetEntry("vectorToField/roll").SetDouble(EctoMath::radiansToDegrees(rotation.X().value()));

    return {translation, rotation};
}

int LimeLight::getBestTagId() const{
    auto value = camera->GetEntry("tid").GetDouble(-1);
    return value;
}

std::string LimeLight::getName() const {
    return name;
}

void LimeLight::setLeds(LEDState state) {
    switch (state) {
        case LEDState::kON:
            camera->GetEntry("ledMode").SetDouble(3);
            break;
        case LEDState::kOff:
            camera->GetEntry("ledMode").SetDouble(0);
            break;
        case LEDState::kBlink:
            camera->GetEntry("ledMode").SetDouble(2);
            break;
    }
    this->state = state;
}

double LimeLight::getLatency() const {
    double out = camera->GetEntry("tl").GetDouble(0);
//    table->GetEntry("latency").SetDouble(out);
    return out;
}

LEDState LimeLight::getLeds() const {
    return state;
}

units::radian_t LimeLight::getTargetYaw() const {
    return units::degree_t((camera->GetEntry("tx").GetDouble(0)));
}

units::radian_t LimeLight::getTargetPitch() const {
    return units::degree_t((camera->GetEntry("ty").GetDouble(0)));
}

bool LimeLight::hasTarget() const {
    bool out = camera->GetEntry("tv").GetDouble(0) == 1.0;
//    table->GetEntry("hasTarget").SetBoolean(out);
    return out;
}

void LimeLight::setPipeLine(int id){
    camera->GetEntry("pipeline").SetDouble(id);
}

double LimeLight::getAreaOfImage() const {
    return camera->GetEntry("ta").GetDouble(0);
}

double LimeLight::getTargetingLatency() const {
    return camera->GetEntry("tl").GetDouble(0);
}

int LimeLight::getPipeLine() const {
    return camera->GetEntry("getpipe").GetDouble(-1);
}

std::vector<AprilTag> LimeLight::getAprilTags() const {
    std::vector<AprilTag> out{};
    auto jsonDump = getResultsFromJSON().FiducialResults;
    for (auto &result : jsonDump){
        AprilTag tag;
        units::second_t latency = units::millisecond_t(result.m_TargetLatency);
        tag.c_ts = VisionUtilities::vectorToTransform3d(result.m_CAMERATransform6DTARGETSPACE);
        tag.r_fs = vectorToField<frc::Transform3d>(result.m_ROBOTTransform6DFIELDSPACE, 16.459, 8.23);
        tag.r_ts = VisionUtilities::vectorToTransform3d(result.m_ROBOTTransform6DTARGETSPACE);
        tag.t_cs = VisionUtilities::vectorToTransform3d(result.m_TargetTransform6DCAMERASPACE);
        tag.t_rs = VisionUtilities::vectorToTransform3d(result.m_TargetTransform6DROBOTSPACE);
        tag.tagId = result.m_fiducialID;
        tag.targetInfo.timestamp = frc::Timer::GetFPGATimestamp() - latency;
        tag.targetInfo.hasTargets = hasTarget();
        tag.targetInfo.targetingLatency = latency;
        tag.targetInfo.areaOfImage = getAreaOfImage();
        out.emplace_back(tag);
    }
    return out;

}

void LimeLight::updateTelemetry() {

}
