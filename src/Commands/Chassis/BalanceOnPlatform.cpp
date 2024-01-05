//
// Created by cc on 11/02/23.
//

#include "BalanceOnPlatform.h"

BalanceOnPlatform::BalanceOnPlatform(const std::shared_ptr<EctoSwerve> &swerve) :
    pitchController(5.390625, 0, 0), rollController(1.35, 0, 0) {
    this->swerve = swerve;
    AddRequirements(swerve.get());
}

void BalanceOnPlatform::Initialize() {
    pitchController.Reset();
    rollController.Reset();

    pitchController.SetSetpoint(0);
    rollController.SetSetpoint(0);

    pitchFilter.Reset();
    auto initOut = frc::ChassisSpeeds::FromFieldRelativeSpeeds({-0.001_mps, -0.001_mps, 0.001_rad_per_s}, swerve->getPose().Rotation());
    swerve->setVoltage(initOut);
}

void BalanceOnPlatform::Execute() {
    double maxVal = 2.15625;
    double yaw = swerve->getYaw();

    double pitch_rot = getRotPitch();

    double u_x = std::clamp(pitchController.Calculate(pitch_rot), -maxVal, maxVal);
//    double u_x = std::clamp(bangBangHysterisis(pitch_rot), -maxVal, maxVal);
    table->GetEntry("rawU_x").SetDouble(u_x);

    double rot = 0.14; //en el regional laguna teniamos 0.16 de hysteresis y jalo al inicio pero luego las balanzas se hiban aflojando
    pitch_rot = std::abs(pitch_rot);
    if(pitch_rot < rot && pitch_rot > -rot){
        u_x = 0.0;
    }
    double u_y = 0.0;

    frc::ChassisSpeeds out{};
    out.vx = units::meters_per_second_t(u_x);
    out.vy = units::meters_per_second_t(u_y);
    out = frc::ChassisSpeeds::FromFieldRelativeSpeeds(out.vx , out.vy, units::radians_per_second_t(0), frc::Rotation2d(units::radian_t(yaw)));
    swerve->setVoltage(out);
    table->GetEntry("pitchRate").SetDouble(pitchRate);
    table->GetEntry("pitchRot").SetDouble(pitch_rot);
    table->GetEntry("u_y").SetDouble(u_y);
    table->GetEntry("u_x").SetDouble(u_x);
    table->GetEntry("PitchRotated").SetDouble(pitch_rot);
    table->GetEntry("out/vx").SetDouble(out.vx.value());
    table->GetEntry("out/vy").SetDouble(out.vy.value());
    table->GetEntry("out/omega").SetDouble(out.omega.value());
    prevPitch = pitch_rot;
}

void BalanceOnPlatform::End(bool interrupted){
    swerve->setVoltage({});
}

double BalanceOnPlatform::getRotPitch() const{
    auto yaw = swerve->getYaw();
    auto pitch = swerve->getPitch();
    auto roll = swerve->getRoll();

    table->GetEntry("raw/pitch").SetDouble(pitch);
    table->GetEntry("raw/roll").SetDouble(roll);
    table->GetEntry("raw/yaw").SetDouble(yaw);

    return pitch * std::cos(-yaw) - roll * std::sin(-yaw);

}

double BalanceOnPlatform::bangBangHysterisis(double in){
    if (in < -0.2){
        return 1.2;
    }else if (in > 0.2){
        return -1.2;
    }else {
        return 0;
    }

}

bool BalanceOnPlatform::IsFinished() {
    return false;
    double error = std::abs(getRotPitch());
    std::cout << fmt::format("error: {} pitch rate: {}", error, pitchRate) << std::endl;
    return error < 0.1 || std::abs(pitchRate) > 0.02; // TODO Tune
}