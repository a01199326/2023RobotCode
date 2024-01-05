//
// Created by cc on 7/01/23.
//

#include "DoosanPumaSMX3100.h"
#include <wpi/json.h>
#include <random>
#include "Commands/Manipulator/ManipulatorState.h"
#include "Commands/Manipulator/IntakePiece/IntakePiece.h"

DoosanPumaSMX3100::DoosanPumaSMX3100() : EctoRobot("DoosanPumaSMX3100") {
    ;
}

void DoosanPumaSMX3100::disabledInit() {
    ;
}

void DoosanPumaSMX3100::disabledUpdate() {
    ;
}

void DoosanPumaSMX3100::robotInit() {

    table->GetEntry("setLeds").SetBoolean(false);
    EctoSwerveConfig swerveConfig;
    // from bumper edge to bumper edge.
    swerveConfig.length =   23_in; //meters 18.530 in
    swerveConfig.width =    23_in; // meters 18.655 in
    swerveConfig.wheelCircumference = 0.0508 * 2 * M_PI;
    double gearRatio = (42.0 / 13.0) * (18.0 / 26.0) * 3.0;
    swerveConfig.gearRatio = gearRatio;

    VisionManagerConfig visionConfig;
    visionConfig.statesStdDevs = wpi::array<double, 3>{0.01, 0.01, 0.01};
    visionConfig.gyroEncoderStdDevs = wpi::array<double, 1>{0.1};
    visionConfig.autoVisionStdDevs = wpi::array<double, 3>{0.1, 0.1, 0.1};
    visionConfig.teleopVisionStdDevs = wpi::array<double, 3>{0.4, 0.4, 0.4};
    visionConfig.robotPose = frc::Pose2d{{0_m, 0_m}, 0_rad};
    visionConfig.robotYaw = frc::Rotation2d{0_rad};
    visionConfig.tagPoses.tag16h5 = {
            {1, {610.77_in, 42.19_in, {180_deg}}},
            {2, {610.77_in, 108.18_in, {180_deg}}},
            {3, {610.77_in, 174.19_in, {180_deg}}},
            {4, {636.96_in, 265.74_in, {180_deg}}},
            {5, {14.25_in, 265.74_in, {0_deg}}},
            {6, {40.45_in, 174.19_in, {0_deg}}},
            {7, {40.45_in, 108.19_in, {0_deg}}},
            {8, {40.45_in, 41.19_in, {0_deg}}},

    };

    visionConfig.visionStdDevs = {
            {0.0001, 10},
            {0.01, 0.5},
            {0.15, 0.1},// this is based on the percentage of image the tag takes up,
            {0.25, 0.05},// if the number is smaller, then the image is further away.
            {0.45, 0.04},
            {1.4, 0.03},
            {1.6, 0.025},
            {2.5, 0.02},
            {6.5, 0.0088}


    };


    visionConfig.targetHeight = units::centimeter_t(263);
    visionConfig.fieldToTarget = frc::Pose2d(8.25_m, 4.07_m, {0_deg});
    visionConfig.cameraPoses = {
            frc::Pose3d(5.89_cm, -22.26_cm, 42.63_cm, {0_deg, 25_deg, 0_deg}),
            frc::Pose3d(-6.8_cm, 17.5_cm, 41.4_cm + 1.5_in, {0_deg, 25_deg, -180_deg}),
    };
    visionConfig.midPoleHigh = 0.61_m;
    visionConfig.tagHeights = 0.61_m;

    PIDWristConfig wristConfig;
    wristConfig.pidConfig.p = 12.8;
    wristConfig.pidConfig.i = 0;
    wristConfig.pidConfig.d = 1.5 / 2.7;
    wristConfig.motors = {motorManager.getMotor("wrist")};
    wristConfig.isInverted = {true};
    wristConfig.invertSensor = true;
    wristConfig.rampRate = 0.01;
    wristConfig.gearReduction = 300;
    wristConfig.sensorReduction = 1.3;
    wristConfig.currentLimit = 40_A;
    wristConfig.enableReverseSoftLimit = false;
    wristConfig.enableForwardSoftLimit = false;
    wristConfig.forwardSoftLimit = 2.3_rad;
    wristConfig.reverseSoftLimit = -2.3_rad;
    wristConfig.sensorOffset = 0.1216;
    wristConfig.maxVel = 24.0_rad_per_s;
    wristConfig.maxAccel = 20.0_rad_per_s_sq;
    wristConfig.intakeMaxVel = 12.0_rad_per_s;
    wristConfig.intakeMaxAccel = 12.0_rad_per_s_sq;
    wristConfig.pidDistTol = 0.01_rad;
    wristConfig.pidVelTol = 1_rad_per_s;
//    wristConfig.ff = {0.55029_V, 0.82901_V, 0.46626_V / 1_rad_per_s, 0.042033_V / 1_rad_per_s_sq};
    wristConfig.ff = {0.095273_V, 1.8007_V / 1_rad_per_s, 0.034144_V / 1_rad_per_s_sq};

    PIDTelescopicConfig telescopicConfig;
    telescopicConfig.pidConfig.p = 92.0;
    telescopicConfig.pidConfig.i = 0.45;
    telescopicConfig.pidConfig.d =  6.7437 / 3.8;
    telescopicConfig.freeFF = {0.70367_V, 0.0_V, 2.6792_V / 1_mps, 0.3725_V / 1_mps_sq};
    telescopicConfig.motors = {motorManager.getMotor("telescopic"),
                               motorManager.getMotor("telescopicFollower"),
                               motorManager.getMotor("telescopicSecondFollower")};
    telescopicConfig.isInverted = {false, true, true};
    telescopicConfig.reverseSoftLimit = 0_m;
    telescopicConfig.forwardSoftLimit = 1.1_m;
    telescopicConfig.currentLimit = 60_A;
    telescopicConfig.rampRate = 0.01;
    telescopicConfig.gearReduction = 8.444444;
    telescopicConfig.pulleyDiameter = 47.75_mm;
    telescopicConfig.enableReverseSoftLimit = true;
    telescopicConfig.enableForwardSoftLimit = true;
    telescopicConfig.maxGoingUpTeleopVel = 7.0_mps;
    telescopicConfig.maxGoingUpTeleopAccel = 7.8_mps_sq;
    telescopicConfig.maxGoingDownTeleopVel = 6.0_mps;
    telescopicConfig.maxGoingDownTeleopAccel = 7.0_mps_sq;
    telescopicConfig.maxGoingUpAutoVel = 5.0_mps;
    telescopicConfig.maxGoingUpAutoAccel = 3.5_mps_sq;
    telescopicConfig.maxGoingDownAutoVel = 5.5_mps;
    telescopicConfig.maxGoingDownAutoAccel = 3.5_mps_sq;
    telescopicConfig.pidDistTol = 0.055_m;
    telescopicConfig.pidVelTol = 1.0_mps;
    telescopicConfig.stages = 2;

    ManipulatorConfig manipulatorConfig;
    manipulatorConfig.motors = {motorManager.getMotor("manipulator")};
//    manipulatorConfig.pistons = {pcm.getPiston("manipulator")};
    manipulatorConfig.isInverted = {true};
    manipulatorConfig.currentLimit = 40_A;
    manipulatorConfig.breakingOnIdle = true;
    manipulatorConfig.idlePercent = -0.1;
    manipulatorConfig.cubeCurrentThreshold = 20_A;
    manipulatorConfig.coneCurrentThreshold = 20_A;
    manipulatorConfig.manipulatorWidth = 0.34;
    manipulatorConfig.coneRadius = 0.025;

    PIDJointConfig armSteerConfig;
    armSteerConfig.pidConfig.p = 15.3;
    armSteerConfig.pidConfig.i = 0.0;
    armSteerConfig.pidConfig.d = 0.59082 / 2.8;
    armSteerConfig.ff = {1.527_V, 0.355_V / 1_rad_per_s, 0.034_V / 1_rad_per_s_sq};
    armSteerConfig.motors = {motorManager.getMotor("arm"),
                             motorManager.getMotor("armFollower")};
    armSteerConfig.isInverted = {false, true};
    armSteerConfig.invertSensor = true;
    armSteerConfig.rampRate = 0.01;
    armSteerConfig.gearReduction = 49.86;//not precise due to belt
    armSteerConfig.currentLimit = 60_A;
    armSteerConfig.enableReverseSoftLimit = false;
    armSteerConfig.enableForwardSoftLimit = false;
    armSteerConfig.forwardSoftLimit = 180_deg;
    armSteerConfig.reverseSoftLimit = -180_deg;
    armSteerConfig.analogOffset = -2.0415;
    armSteerConfig.maxVel = 10.5_rad_per_s;
    armSteerConfig.maxAccel = 15.0_rad_per_s_sq;
    armSteerConfig.pidDistTol = 0.008_rad;
    armSteerConfig.pidVelTol = 1.1_rad_per_s;

    PWMColorSensorConfig colorSensorConfig;
    colorSensorConfig.DIOPin = 8;
    colorSensorConfig.pulsePeriod = 1.0 / 4095.0;
    colorSensorConfig.bound = 300.0 / 4095;
    colorSensorConfig.coneThreshold = 0.6;
    colorSensorConfig.cubeThreshold = 0.2;

    LedsWithDIOConfig ledsWithDioConfig;
    ledsWithDioConfig.pinIDs = {4, 5, 3};
    ledsWithDioConfig.colorsToInts = {
            {"green", 0},
            {"red", 1},
            {"orange", 2},
            {"purple", 3},
            {"arcWhite", 4},
            {"wristWhite", 5},
            {"bothWhite", 6},
            {"off", 7}
    };

    swerve = std::make_shared<EctoSwerve>(swerveConfig);
//    pdh = std::make_shared<PowerDistributionHub>(1, frc::PowerDistribution::ModuleType::kRev);
    CameraInfo leftInfo;
    leftInfo.globalCameraId = 0;
    leftInfo.resolution = {640, 480};
    leftInfo.verticalFov = 0_deg;
    leftInfo.software = Software::kLIMELIGHT;
    leftInfo.cameraPose = {{40_cm, 14_cm, 30_cm}, {180_deg, 0_deg, 0_deg}};
    leftInfo.ledState = LEDState::kOff;
    leftInfo.cameraName = "center";

    auto leftCamera = std::make_shared<LimelightSource>(leftInfo, swerve);
    std::vector<std::shared_ptr<VisionSource>> sources = {leftCamera};

//    visionSource->setLeds(LEDState::kOff, 1);
//    visionSource->setLeds(LEDState::kOff, 0);
    visionManager = std::make_shared<VisionManager>(swerve, sources, visionConfig);
    visionManager->setPipeline(0, 1);
    wrist = std::make_shared<PIDWrist>(wristConfig);
    manipulator = std::make_shared<Manipulator>(manipulatorConfig);
    telescopic = std::make_shared<PIDTelescopic>(telescopicConfig);
    armSteer = std::make_shared<PIDJoint>(armSteerConfig);
    colorSensor = std::make_shared<PWMColorSensor>(colorSensorConfig);

    ledsWithDIO = std::make_shared<LedsWithDIO>(ledsWithDioConfig);

    robotPose = std::make_shared<frc::Field2d>();
    tagPose = std::make_shared<frc::Field2d>();
    sampleField = std::make_shared<frc::Field2d>();

    autoTest = std::make_shared<PathFollowingAutoTest>(
            swerve,
            armSteer,
            telescopic,
            wrist,
            manipulator,
            colorSensor,
            visionManager
            );

    swerve->SetDefaultCommand(EctoSwerveInputHandler(swerve, manipulator, visionManager));


    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
    frc::SmartDashboard::PutData("FusedRobotPose", robotPose.get());

    frc::Translation2d armCoords{};

    inputManager.registerButton(mambaPose, "select", 1);
    inputManager.registerButton(invertedMambaPose, "start", 1);


    inputManager.registerButton(frontIntakeHigh, "B", 1);
    inputManager.registerDPadButton(backIntakeHigh, "right", 1);

    inputManager.registerButton(frontScoreHigh, "Y", 1);
    inputManager.registerDPadButton(backScoreHigh, "up", 1);

    inputManager.registerButton(frontScoreMid, "X", 1);
    inputManager.registerDPadButton(backScoreMid, "left", 1);

    inputManager.registerButton(frontCubeMid, "B", 1);
//    inputManager.registerButton(testingVisionAlignCommand, "B", 0);

    inputManager.registerButton(frontIntakeLow, "A", 1);
    inputManager.registerDPadButton(backIntakeLow, "down", 1);
    inputManager.registerButton(balancePoseFront, "rightJoystick", 1);


    inputManager.registerAxis(manualIntake, "rightTrigger", 1);
    inputManager.registerAxis(manualOuttake, "leftTrigger", 1);

    inputManager.registerButton(frontIntakeUpRightCone, "rightBumper", 1);
    inputManager.registerButton(backIntakeUpRightCone, "leftBumper", 1);

    inputManager.registerButton(doubleSubStation, "leftJoystick", 1);

    initStateChooser.SetDefaultOption("frontIdle", ManipulatorStates::front().idle);
    initStateChooser.AddOption("backIdle", ManipulatorStates::back().idle);

    frc::SmartDashboard::PutData("Idle Chooser", &initStateChooser);

    initState = initStateChooser.GetSelected();

    armSteer->set(initState.jointAngle);
    telescopic->set(0.0);
    telescopic->resetToZero();
    telescopic->resetController(0.0_m);
    wrist->set(initState.wristAngle);
    manipulator->idle();

    ledsTable->GetEntry("init").SetBoolean(true);

}

void DoosanPumaSMX3100::robotUpdate() {
    robotPose->SetRobotPose(swerve->getPose());
    bool isEnable = frc::DriverStation::IsEnabled();
    bool piece = colorSensor->getCurrentPiece() == ScorePiece::CONE;
    bool inIdle = wrist->inTol(1.5_deg) && armSteer->inTol(1.5_deg);
    bool error = wrist->getAngle() < -2.39_rad;
    auto idleFront = ManipulatorStates::front().idle;
    auto idleBack = ManipulatorStates::back().idle;
    bool wristInTol = wrist->inTolOf(1.5_deg, idleFront.wristAngle) || wrist->inTolOf(2.5_deg, idleBack.wristAngle) || wrist->inTolOf(5_deg, 2.31_rad) || wrist->inTolOf(5_deg, -2.28_rad);
    bool armSteerInTol = armSteer->inTolOf(1.5_deg, idleFront.jointAngle) || armSteer->inTolOf(1.5_deg, idleBack.jointAngle);


    ledsTable->PutString("RobotSide", robotSide == RobotSide::kFront ? "front" : "back");
    ledsTable->GetEntry("enable").SetBoolean(isEnable);
    ledsTable->GetEntry("Piece").SetBoolean(piece);
    ledsTable->GetEntry("inIdle").SetBoolean(inIdle);
    ledsTable->GetEntry("error").SetBoolean(error);
    ledsTable->GetEntry("wristInTol").SetBoolean(wristInTol);
    ledsTable->GetEntry("arcInTol").SetBoolean(armSteerInTol);
    bool aligned = ledsTable->GetEntry("aligned").GetBoolean(false);


    ledsWithDIO->setEffects(ledsWithDIO->decideEffect(true,
                                                      isEnable,
                                                      piece,
                                                      inIdle,
                                                      error,
                                                      wristInTol,
                                                      armSteerInTol,
                                                      aligned));

    auto yaw = swerve->getYaw();
    auto pitch = swerve->getPitch();
    auto roll = swerve->getRoll();

    frc::SmartDashboard::PutNumber("tuneBalance", (pitch * std::cos(-yaw) - roll * std::sin(-yaw)));


//    auto offset = visionManager->getYOffsetToTarget(0);
//    auto xOffset = visionManager->getXOffsetToTarget(0);

//    if (offset.has_value()){
//        auto y = offset->value();
//        frc::SmartDashboard::PutNumber("yoffset", y);
//        frc::SmartDashboard::PutNumber("magicOut",  (2.0749 * (std::pow(y, 2)) + (1.6712 * y) - 0.0453));
//        frc::SmartDashboard::PutNumber("xoffset", xOffset->value());
//    }
}

void DoosanPumaSMX3100::autoInit() {
    initState = initStateChooser.GetSelected();
    autoCommand = autoChooser.GetSelected();
    commandScheduler.Schedule(autoTest.get());
    visionManager->setVisionMinMaxStdDevs(1.0, 2.0);
    telescopic->resetToZero();
    telescopic->resetController(0.0_m);
    armSteer->setConstraints(7.0_rad_per_s, 5.5_rad_per_s_sq);
    wrist->setConstraints(20.0_rad_per_s, 13.0_rad_per_s_sq);
    armSteer->set(initState.jointAngle);
    armSteer->resetController();
    wrist->set(initState.wristAngle);
    wrist->resetController();
}

void DoosanPumaSMX3100::autoUpdate() {
    ;
}

void DoosanPumaSMX3100::teleopInit() {
    visionManager->ignoreVision(false);
    visionManager->setVisionMinMaxStdDevs(0.05, 0.1);
    armSteer->setConstraints(8.5_rad_per_s, 8.0_rad_per_s_sq);
    wrist->setConstraints(24.0_rad_per_s, 20.0_rad_per_s_sq);


//    armSteer->resetController();
//    wrist->resetController();
    telescopic->resetController(telescopic->getHeight());
//    visionManager->setPipeline(0, colorSensor->getCurrentPiece() == ScorePiece::CUBE ? 2 : 1);
//    intakeCube.WhileTrue(SetManipulator(manipulator, 1.0, true).ToPtr());
//    intakeCone.WhileTrue(SetManipulator(manipulator, 0.6, false).ToPtr());
//    releaseCone.WhileTrue(SetManipulator(manipulator, -1.0, true).ToPtr());
//    releaseCube.WhileTrue(SetManipulator(manipulator, -0.5, true).ToPtr());

}

void DoosanPumaSMX3100::teleopUpdate() {
//
    currentPiece = colorSensor->getCurrentPiece();
    if (resetOdoToPoint.get()){
        swerve->resetOdometry({14.664_m, 5.0_m, {0.0_deg}});
    }

    if (currentPiece != prevCurrentPiece) {
        commandScheduler.CancelAll();
        commandScheduler.GetActiveButtonLoop()->Clear();
        manualOuttake.WhileTrue(SetManipulator(manipulator, 0.13, true).ToPtr());
        manualIntake.WhileTrue(SetManipulator(manipulator, -0.13, true).ToPtr());
        frontIntakeUpRightCone.WhileTrue(SetManipulator(manipulator, -1.0, true).ToPtr());
        backIntakeUpRightCone.WhileTrue(SetManipulator(manipulator, 1.0, true).ToPtr());
//        autoBalance.WhileTrue(BalanceOnPlatform(swerve).ToPtr());
//        autoBalance.WhileTrue(ResetWheelStateToZero(swerve, false).ToPtr());
        manipulator->setPieceThreshold(currentPiece);
//        visionManager->setPipeline(0, currentPiece == ScorePiece::CONE ? 1 : 2);

        VisionAlignConfig vaConfig;
        vaConfig.yPIDConfig.p = 7.2;
        vaConfig.yPIDConfig.i = 0.3;
        vaConfig.yPIDConfig.d = 0.001;
        vaConfig.xPIDConfig.p = 1.4;
        vaConfig.xPIDConfig.i = 0.01;
        vaConfig.xPIDConfig.d = 0.000005;
        vaConfig.yawPIDConfig.p = 7.2;
        vaConfig.yawPIDConfig.i = 0.01;
        vaConfig.yawPIDConfig.d = 0.0003;
        vaConfig.maxVel = 4.3_mps;
        vaConfig.maxAccel = 2.5_mps_sq;
        vaConfig.maxAngularVel = 4.35_rad_per_s;
        vaConfig.maxAngularAccel = 3.05_rad_per_s_sq;
        testingVisionAlignCommand.WhileTrue(VisionAlign(swerve, visionManager, manipulator, vaConfig, currentPiece).ToPtr());

//
//        std::vector<frc::Translation2d> coneSnapPoses = {
//
//                //red side
//
//                {14.77_m, 0.51_m},//cone
//                {14.77_m, 1.63_m},//cone
//
//                {14.77_m, 2.19_m},//cone
//                {14.77_m, 3.31_m},//cone
//
//                {14.77_m, 3.87_m},//cone
//                {14.728_m, 4.98_m},//cone
//
//                //blue side
//
//                {1.3_m, 0.48_m},
//                {1.3_m, 1.61_m},
//
//                {1.3_m, 2.16_m},
//                {1.3_m, 3.27_m},
//
//                {1.3_m, 3.86_m},
//                {1.3_m, 4.61_m}
//        };
//
//
//        std::vector<frc::Translation2d> cubeSnapPoses = {
//                //red Side
//
//                {14.77_m, 1.07_m},//cube
//                {14.77_m, 2.75_m},//cube
//                {14.77_m, 4.42_m},//cube
//
//                //blue side
//
//                {1.3_m, 1.06_m},//cube
//                {1.3_m, 2.71_m},//cube
//                {1.3_m, 4.43_m},//cube
//
//        };
//
//        SnapToPointConfig pointConfig;
//        pointConfig.pos_p = 6.5;
//        pointConfig.pos_i = 0.085;
//        pointConfig.pos_d = 0.005;
//        pointConfig.theta_p = 5;
//        pointConfig.theta_i = 0.1;
//        pointConfig.theta_d = 0.0003;
//        pointConfig.maxVel = 4.35_mps;
//        pointConfig.maxAccel = 3.0_mps_sq;
//        pointConfig.cubePoints = cubeSnapPoses;
//        pointConfig.conePoints = coneSnapPoses;
//


//        snapToPoint.WhileTrue(MoveToPoint(swerve, {5.4_m, 5.3_m, {0_deg}}, moveToPointConfig).ToPtr());
//        snapToPoint.WhileTrue(SnapToPoint(swerve, pointConfig, currentPiece).ToPtr());

        balancePoseFront.OnTrue(
                SetRobotState(armSteer, wrist, telescopic, ManipulatorStates::back().tiltStation).ToPtr()
                );

        doubleSubStation.OnTrue(
                IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::back().intakeDoubleStation, ManipulatorStates::front(), currentPiece).ToPtr()
                );

        mambaPose.OnTrue(
                frc2::SequentialCommandGroup(
                        frc2::InstantCommand([w = wrist]{w->setDefaultConstraints();}),
                        frc2::InstantCommand([vm = visionManager, cp = currentPiece]{vm->setPipeline(0, cp == ScorePiece::CONE ? 1 : 2);}),
                        SetRobotState(armSteer, wrist, telescopic, ManipulatorStates::front().idle)
//                        SetManipulator(manipulator, 0, true, true),
//                        frc2::WaitCommand(0.3_s),
//                        SetManipulator(manipulator, 0, false, true)
                ).ToPtr()
        );

        invertedMambaPose.OnTrue(
                frc2::SequentialCommandGroup(
                        frc2::InstantCommand([w = wrist]{w->setDefaultConstraints();}),
                        frc2::InstantCommand([vm = visionManager, cp = currentPiece]{vm->setPipeline(0, cp == ScorePiece::CONE ? 1 : 2);}),
                        SetRobotState(armSteer, wrist, telescopic, ManipulatorStates::back().idle)
//                        SetManipulator(manipulator, 0, true, true),
//                        frc2::WaitCommand(0.3_s),
//                        SetManipulator(manipulator, 0, false, true)
                ).ToPtr()
        );

        frontIntakeLow.OnTrue(
                frc2::ConditionalCommand(
                        IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::front().intakeConeFloor, ManipulatorStates::front(), currentPiece),
                        IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::front().intakeCubeFloor, ManipulatorStates::front(), currentPiece),
                        {[piece = currentPiece]{return piece == ScorePiece::CONE;}}

                ).ToPtr()
        );

        backIntakeLow.OnTrue(
                frc2::ConditionalCommand(
                        IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::back().intakeConeFloor, ManipulatorStates::back(), currentPiece),
                        IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::back().intakeCubeFloor, ManipulatorStates::back(), currentPiece),
                        {[piece = currentPiece]{return piece == ScorePiece::CONE;}}

                ).ToPtr()
        );

//        frontIntakeUpRightCone.OnTrue(
//                IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::front().intakeConeUpRight, ManipulatorStates::front(), currentPiece).ToPtr()
//                );
//
//        backIntakeUpRightCone.OnTrue(
//                IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::back().intakeConeUpRight, ManipulatorStates::back(), currentPiece).ToPtr()
//                );

        frontScoreHigh.OnTrue(
                ManipulateManipulator(armSteer, wrist, telescopic, manipulator, colorSensor, ManipulatorStates::front(), ScoreHeight::HIGH, currentPiece).ToPtr()
        );

        backScoreHigh.OnTrue(
                ManipulateManipulator(armSteer, wrist, telescopic, manipulator, colorSensor, ManipulatorStates::back(), ScoreHeight::HIGH, currentPiece).ToPtr()
        );

        frontScoreMid.OnTrue(
                ManipulateManipulator(armSteer, wrist, telescopic, manipulator, colorSensor, ManipulatorStates::front(), ScoreHeight::MID, currentPiece).ToPtr()
        );

        backScoreMid.OnTrue(
                ManipulateManipulator(armSteer, wrist, telescopic, manipulator, colorSensor, ManipulatorStates::back(), ScoreHeight::MID, currentPiece).ToPtr()
        );

        frontIntakeHigh.OnTrue(
                IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::front().intakeStation, ManipulatorStates::front(), currentPiece, true).ToPtr()
        );

        backIntakeHigh.OnTrue(
                IntakePiece(armSteer, telescopic, wrist, manipulator, visionManager, ManipulatorStates::back().intakeStation, ManipulatorStates::back(), currentPiece, true).ToPtr()
        );
//        if (robotSide == RobotSide::kFront) {
//
//        }
//        if (robotSide == RobotSide::kBack) {
//
//        }
        prevCurrentPiece = currentPiece;
    }



// manual
//    wrist->set(frc::SmartDashboard::GetNumber("wrist/set", 0));
//    if(!cubeButton.get() && !coneButton.get() && !releaseCone.get() && !releaseCube.get()){
//        manipulator->setMotor(0, MotorControlMode::Percent);
//    }
//    armSteer->set(frc::SmartDashboard::GetNumber("arm/set", 0));
//    telescopic->set(frc::SmartDashboard::GetNumber("telescopic/set", 0));
//    elevator->set(frc::SmartDashboard::GetNumber("elevator", 0));

}

//void DoosanPumaSMX3100::teleopUpdate(){
//    armSteer->set((frc::SmartDashboard::GetNumber("1-armSteerGoal", 0)));
//    wrist->set((frc::SmartDashboard::GetNumber("1-wristGoal", 0)));
//}

void DoosanPumaSMX3100::simInit() {
    log->info("This should not be running");
//    swerveSim = std::make_unique<EctoSwerveSim>(swerve);
//    managerHandler.addManager(*swerveSim);
//
//    swerveSim->resetPose({7.3298_m, 1.7478_m, -91.5_deg});

//    flywheelSim = std::make_unique<EctoFlywheelSim>(
//            motorManager.getSimulatedMotor("shooterMotor"),
//            frc::DCMotor::NEO(2),
//            1,
//            0.019842, 0.0022638);

//    managerHandler.addManager(*flywheelSim);

//    elevatorSim = std::make_unique<EctoElevatorSim>(
//            motorManager.getSimulatedMotor("frontClimber"),
//            frc::DCMotor::NEO(2),
//            10_kg,
//            0.04064_m,
//            20.94
//    );

//    managerHandler.addManager(*elevatorSim);
}

void DoosanPumaSMX3100::simUpdate() {

}
