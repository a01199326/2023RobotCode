//
// Created by cc on 7/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_DOOSANPUMASMX3100_H
#define BOTBUSTERS_REBIRTH_DOOSANPUMASMX3100_H


#include "Core/EctoRobot.h"
#include "Core/Robot/RobotState.h"

//Vision
#include "Core/VisionManager/VisionManager.h"
#include "Core/VisionManager/Sources/PhotonvisionSource/PhotonVisionSource.h"
#include "Core/VisionManager/Sources/LimeLightSource/LimelightSource.h"

//Systems
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/EctoSwerve/EctoSwerveInputHandler.h"
#include "Systems/PID/PIDElevator/PIDElevator.h"
#include "Systems/PID/PIDWrist/PIDWrist.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"
#include "Systems/Generic/Manipulator/Manipulator.h"
#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Sensors/EctoColorSensor/PWMColorSensor.h"
#include "Systems/Generic/LedsWithDIO/LedsWithDIO.h"

//Autonomous
#include "Commands/Autonomous/TaxiCommand/TaxiCommand.h"
#include "Commands/Autonomous/PathFollowingAutoTest.h"

#include "Commands/Autonomous/BlueAlliance/Blue1ConeBalance/Blue1ConeBalance.h"

#include "Commands/Autonomous/RedAlliance/RedLink/RedLink.h"

//sample auto
#include "Commands/Autonomous/SampleCommand/SampleAuto.h"


//WPI
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/ConditionalCommand.h>

//wpi

//Simulation
#include "Systems/EctoSwerve/Simulation/EctoSwerveSim.h"
#include "Control/Kinematics/Arm/SimpleArmKinematics.h"

//Commands
#include "Commands/Manipulator/SetManipulator.h"
#include "Commands/Telescopic/SetTelescopic/SetTelescopic.h"
#include "Commands/Joint/SetJoint/SetJoint.h"
#include "Commands/Wrist/SetWrist/SetWrist.h"

#include "Commands/Manipulator/ManipulateManipulator/ManipulateManipulator.h"
#include "Commands/Manipulator/SetRobotState/SetRobotState.h"
#include "Commands/Chassis/BalanceOnPlatform.h"
#include "Commands/Chassis/SnapToPoint/SnapToPoint.h"
#include "Commands/Autonomous/AutoCommands/VisionAlign/VisionAlign.h"

class DoosanPumaSMX3100 : public EctoRobot {
public:
    DoosanPumaSMX3100();

    void disabledInit() override;

    void disabledUpdate() override;

    void robotInit() override;

    void robotUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

    void teleopInit() override;

    void teleopUpdate() override;

    void simInit() override;

    void simUpdate() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {{EctoMotorType::SparkMax, "front_left_wheel",     7},
                {EctoMotorType::SparkMax, "front_right_wheel",    5},
                {EctoMotorType::SparkMax, "back_left_wheel",      1},
                {EctoMotorType::SparkMax, "back_right_wheel",     3},

                {EctoMotorType::SparkMax, "front_left_steer",     8},
                {EctoMotorType::SparkMax, "front_right_steer",    6},
                {EctoMotorType::SparkMax, "back_left_steer",      2},
                {EctoMotorType::SparkMax, "back_right_steer",     4},

                {EctoMotorType::SparkMax, "telescopic", 10},
                {EctoMotorType::SparkMax, "telescopicFollower", 11},
                {EctoMotorType::SparkMax, "telescopicSecondFollower", 12},

                {EctoMotorType::SparkMax, "arm",                    13},
                {EctoMotorType::SparkMax, "armFollower",            14},


                {EctoMotorType::SparkMax, "manipulator",                    15},

                {EctoMotorType::SparkMax, "wrist",               16},
        };
    }
    std::list<PistonInfo> getPistonConfig(){
        return {
                {"manipulator", 21, frc::PneumaticsModuleType::REVPH, 6, 5}// originali 1, 0 the 14 15, 4, 5
        };
    }
private:
    enum class RobotSide{
        kFront,
        kBack
    };

    //managers
    InputManager &input = InputManager::getInstance();
    PCMManager &pcm = PCMManager::getInstance();

    //systems
    std::shared_ptr<EctoSwerve> swerve;
    std::shared_ptr<VisionManager> visionManager;
    std::shared_ptr<PIDWrist> wrist;
    std::shared_ptr<PIDTelescopic> telescopic;
    std::shared_ptr<Manipulator> manipulator;
    std::shared_ptr<PIDJoint> armSteer;
    std::shared_ptr<PWMColorSensor> colorSensor;

    std::shared_ptr<LedsWithDIO> ledsWithDIO;

    std::shared_ptr<PathFollowingAutoTest> autoTest;

    //nt
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table =
            ntInstance.GetTable("1. Doosan Puma SMX 3100 Machining Center"),

            testCamera = ntInstance.GetTable("limelight-center"),
            ledsTable = ntInstance.GetTable("2. Leds");

    //RobotPose buttons
    EctoButton
//    Home /
    mambaPose,
    invertedMambaPose,

//    scoreHigh, /
    frontScoreHigh,
    backScoreHigh,

//    scoreMid, /
    frontScoreMid,
    backScoreMid,

//    scoreCubeMid,
    frontCubeMid,
    backCubeMid,

//    scoreCubeHigh,
    frontCubeHigh,
    backCubeHigh,

//    intakeHigh,
    frontIntakeHigh,
    backIntakeHigh,

    frontIntakeUpRightCone,
    backIntakeUpRightCone,

    intakeTippedCone,
//    intakeScoreLow, /
    frontIntakeLow,
    backIntakeLow,

    shootCubes,

    frontRobotSide,
    backRobotSide,

    autoBalance,
    autoAlign,

    balancePoseFront,
    balancePoseBack,

    snapToPoint,

    resetOdoToPoint,

    doubleSubStation,
    testingVisionAlignCommand
    ;

    //Manipulator Actions
    EctoButton
    intakeCube,
    intakeCone;

    JoystickAxisExpo releaseCube{0.2, 0.2}, intakePiece{0.2, 0.2},
            manualIntake{0.2, 0.2},
            manualOuttake{0.2, 0.2}
    ;

    RobotSide robotSide{RobotSide::kFront};
    RobotSide lastRobotSide{RobotSide::kBack};

    ScorePiece currentPiece = ScorePiece::EMPTY;
    ScorePiece prevCurrentPiece = ScorePiece::OVERLOAD;


    double i{0};

    std::shared_ptr<frc::Field2d> robotPose;
    std::shared_ptr<frc::Field2d> tagPose;
    std::shared_ptr<frc::Field2d> sampleField;
    frc::SendableChooser<frc2::Command *> autoChooser;
    frc::SendableChooser<State> initStateChooser;
    State initState{};
    frc2::Command *autoCommand{};

    std::shared_ptr<TaxiCommand> leftTaxi, rightTaxi;
    std::shared_ptr<SampleAuto> sampleAuto;

    std::shared_ptr<frc::DoubleSolenoid> testPiston;
    //test
    std::unique_ptr<EctoSwerveSim> swerveSim;
    std::shared_ptr<SimpleArmKinematics> armKinematics;



};


#endif //BOTBUSTERS_REBIRTH_DOOSANPUMASMX3100_H
