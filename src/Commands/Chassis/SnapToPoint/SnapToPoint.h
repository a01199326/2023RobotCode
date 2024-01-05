//
// Created by cc on 20/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_SNAPTOPOINT_H
#define BOTBUSTERS_REBIRTH_SNAPTOPOINT_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/VisionManager/VisionManager.h"
#include "Commands/Manipulator/ManipulateManipulator/ManipulateManipulator.h"

struct SnapToPointConfig {
    double pos_p{0}, pos_i{0}, pos_d{0};
    double theta_p{0}, theta_i{0}, theta_d{0};

    units::meters_per_second_t maxVel{1_mps};
    units::meters_per_second_squared_t maxAccel{0.5_mps_sq};

    units::radians_per_second_t maxAngularVel{5_rad_per_s};
    units::radians_per_second_squared_t maxAngularAccel{5_rad_per_s_sq};

    std::vector<frc::Translation2d> cubePoints;
    std::vector<frc::Translation2d> conePoints;
};

class SnapToPoint : public frc2::CommandHelper<frc2::CommandBase, SnapToPoint> {
public:
    SnapToPoint(const std::shared_ptr<EctoSwerve> &swerve, const SnapToPointConfig &config, ScorePiece scorePiece);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    void updateTelemetry();

    std::optional<frc::Pose2d> getClosestPose(const std::vector<frc::Translation2d> &points, const frc::Pose2d currentPose);

private:
    SnapToPointConfig config;

    std::shared_ptr<EctoSwerve> swerve;
    frc::Pose2d targetPose;

    std::vector<frc::Translation2d> points;

    frc::ProfiledPIDController<units::meter> xController, yController;
    frc::ProfiledPIDController<units::radians> thetaController;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table =
            ntInstance.GetTable("MoveToPoint");

    frc::ChassisSpeeds out;
    units::meter_t distToTarget;
    ScorePiece scorePiece;


};


#endif //BOTBUSTERS_REBIRTH_SNAPTOPOINT_H
