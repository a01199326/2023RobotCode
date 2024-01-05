//
// Created by cc on 04/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONALIGN_H
#define BOTBUSTERS_REBIRTH_VISIONALIGN_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>


#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/Generic/Manipulator/Manipulator.h"

#include "Core/VisionManager/VisionManager.h"
#include "Core/VisionManager/Sources/LimeLightSource/LimeLight.h"
#include "Core/Robot/RobotState.h"

#include <limits>

struct VisionAlignConfig{
    PIDConfig xPIDConfig;
    PIDConfig yPIDConfig;
    PIDConfig yawPIDConfig;

    units::meters_per_second_t maxVel{1_mps};
    units::meters_per_second_squared_t maxAccel{1_mps_sq};
    units::radians_per_second_t maxAngularVel{1_rad_per_s};
    units::radians_per_second_squared_t maxAngularAccel{1_rad_per_s_sq};

};

class VisionAlign : public frc2::CommandHelper<frc2::CommandBase, VisionAlign> {
public:
	VisionAlign(const std::shared_ptr<EctoSwerve> &swerve,
                const std::shared_ptr<VisionManager> &visionManager,
                const std::shared_ptr<Manipulator> &manipulator,
                const VisionAlignConfig &config, ScorePiece currentPiece);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

    void updateTelemetry();


private:
    double magicToReal(double magic) const;

    bool isReady() const;

	std::shared_ptr<EctoSwerve> swerve;
    std::shared_ptr<VisionManager> visionManager;
    std::shared_ptr<Manipulator> manipulator;
    VisionAlignConfig config;
    ScorePiece gamePiece;

    frc::ChassisSpeeds out;

    units::radian_t yawGoal{};
    units::meter_t yGoal{};
    units::meter_t xGoal{};

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table =
            ntInstance.GetTable("VisionAlign");

    std::shared_ptr<LimeLight> camera;

    frc::ProfiledPIDController<units::radians> yawController;
    frc::ProfiledPIDController<units::meters> yController;
    frc::ProfiledPIDController<units::meters> xController;

};


#endif //BOTBUSTERS_REBIRTH_VISIONALIGN_H
