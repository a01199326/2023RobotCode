//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H

#include "frc2/command/CommandBase.h"
#include "frc2/command/CommandHelper.h"

#include "EctoSwerve.h"
#include "Core/EctoModule/System.h"
#include <Core/EctoInput/InputManager.h>
#include "Core/VisionManager/VisionManager.h"
#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Buttons/ToggleButton.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/controller/PIDController.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include "Systems/Generic/Manipulator/Manipulator.h"
#include <frc/filter/MedianFilter.h>

#include "Core/VisionManager/Sources/LimeLightSource/LimeLight.h"
#include <frc/Timer.h>

#include "Utilities/BooleanUtils/LatchedBool/LatchedBool.h"
#include "Utilities/BooleanUtils/TimeDelayedBool/TimeDelayedBool.h"

class EctoSwerveInputHandler : public frc2::CommandHelper<frc2::CommandBase, EctoSwerveInputHandler> {
public:
	explicit EctoSwerveInputHandler(const std::shared_ptr<EctoSwerve> &swerve,
                                    const std::shared_ptr<Manipulator> &manipulator,
                                    const std::shared_ptr<VisionManager> &visionManager);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<EctoSwerve> swerve;
    std::shared_ptr<Manipulator> manipulator;
    std::shared_ptr<VisionManager> visionManager;
    InputManager& input = InputManager::getInstance();

    void updateTelemetry();

    double magicToReal(double magic) const;

    frc::ChassisSpeeds desiredState(frc::ChassisSpeeds original) const;

    units::second_t swerveDelay{20_ms};

    units::meters_per_second_t maxChassisSpeeds{4.35_mps};
    units::radians_per_second_t maxAngularSpeeds{7.0_rad_per_s};


	/**
	 * NT
	 */
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table, aprilTagTable, ledsTable;

	const double joystickExpo = 0.25;
	const double joystickDeadzone = 0.15;
	
	const double triggerExpo = 0.2;
	const double triggerDeadzone = 0.2;

    JoystickAxisExpo strafeAxis{joystickExpo, joystickDeadzone}, forwardAxis{joystickExpo,
                                                                             joystickDeadzone}, rotationAxis{
            0.25, 0.2};

    JoystickAxisExpo fieldOrientedAxis{triggerExpo, triggerDeadzone}, shootWhileMove{0.2, 0.2};

    EctoButton resetYaw, visionButton, disableFieldOriented, brake, hMode,
            shootAndMoveButton, alignToPiece, alignToTarget, alignYaw, testButton, alignToSingle;
    units::meters_per_second_t maximumInputAcceleration{55.4}; // / 1s
    units::radians_per_second_t maximumInputAngularAcceleration{M_PI * 8.25}; // / 1S
    frc::SlewRateLimiter<units::meters_per_second> xFilter{maximumInputAcceleration / units::second_t(1.0)};
    frc::SlewRateLimiter<units::meters_per_second> yFilter{maximumInputAcceleration / units::second_t(1.0)};
    frc::SlewRateLimiter<units::radians_per_second> thetaFilter{maximumInputAngularAcceleration / units::second_t(1.0)};

    frc::ProfiledPIDController<units::radian> yawPID;
    frc::ProfiledPIDController<units::meter> yAlignPID;
    frc::ProfiledPIDController<units::meter> xAlignPID;
//    frc::ProfiledPIDController<units::meter> xAlignPID;

    double visionOffset{0};

    static bool registeredJoysticks;
    bool lastVisionAlign {false};
    double orbitModeDirection{1};
    double newSetPoint;
    bool enableRobotOriented{false};

    units::radian_t targetYaw{0_rad};
    units::radian_t rawYaw{0_rad};
    units::radian_t singleYaw{-90_deg};

    frc::Pose2d rawPose{{0_m, 0_m}, {0_deg}};

    bool prevYawButtonState{false};
    bool prevAlignToSingleState{false};
    bool prevYAlignButtonState{false};
    bool prevTestButton{false};

    bool isMaintaining{false};
    TimeDelayedBool shouldMaintainHeading;
    LatchedBool resetMaintain;

    frc::Timer pieceTimer;

//    frc::LinearFilter<double> visionPredict = frc::LinearFilter<double>::SinglePoleIIR(0.02, 20_ms);
    frc::SlewRateLimiter<units::radian> yawOffsetLimiter{units::radian_t(0.15*M_PI) / units::second_t(1)};
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
