//
// Created by abiel on 1/2/20.
//

#include "EctoSwerve.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/manual_merge/swerve/SwerveDriveOdometry.h"
#include "frc/manual_merge/swerve/SwerveDrivePoseEstimator.h"
#include <iostream>

EctoSwerve::EctoSwerve(const EctoSwerveConfig &config) : WPISubsystem("EctoSwerve") {
    /*
     * Kinematics init
     */
    kinematics = createKinematics(config.length,
                                  config.width);

    /*
     * estimator init
     */
    estimator = std::make_unique<botbusters::SwerveDrivePoseEstimator<4>>(
            frc::Rotation2d(0, 0), frc::Pose2d(), *kinematics,
            wpi::array<double, 3>{0.069, 0.069, 0.069}, wpi::array<double, 1>{0.0017},
            wpi::array<double, 3>{0.06, 0.06, 0.06}
            );

    PIDConfig steerConfig;
    steerConfig.p = 0.9;
    steerConfig.i = 0.0;
    steerConfig.d = 0.000004;


    std::vector<PIDConfig> wheelPIDs = {
            {0.00012, 0.0, 0.0, 0.000795},//fl//
            {0.000125, 0.0, 0.0, 0.000810},//fr
            {0.00012, 0.0, 0.000002, 0.000814},//bl//
            {0.000125, 0.0, 0.000002, 0.000815}};//br//

    std::vector<frc::SimpleMotorFeedforward<units::meter>> wheelFFs = {
            {0.091264_V, 2.5912_V / 1_mps, 0.071908_V / 1_mps_sq},//fl
            {0.088022_V, 2.9_V / 1_mps, 0.081849_V / 1_mps_sq},//fr
            {0.072978_V, 2.638_V / 1_mps, 0.10995_V / 1_mps_sq},//bl
            {0.088894_V, 3.05_V / 1_mps, 0.074842_V / 1_mps_sq}};//br


    std::vector<std::string> motorPrefixes = {"front_left", "front_right",
                                              "back_left", "back_right"};

    //backup swerve offset per side fl 1.855, fr -2.845, bl 0.32, br -1.354
    std::vector<double> analogOffsets = {
             1.05, //fl//
            -2.905, //fr//
             2.29, //bl//
            -3.12};//br//

    std::vector<double> pctMultiplier = {
        1.0, //fl
        1.0, //fr
        1.0, //bl
        1.0 //br
    };


    std::vector<SwerveModuleConfig> moduleConfigs = {};

    for (size_t i = 0; i < modules.size(); i++) {
        SwerveModuleConfig out;
        out.gearRatio = config.gearRatio;
        out.wheelCircumference = config.wheelCircumference;
        out.steerPID = steerConfig;
        out.wheelPID = wheelPIDs[i];
        out.steerConstraints = {
                units::radians_per_second_t(2 * M_PI * 20),
                units::radians_per_second_squared_t(
                        2.0 * M_PI * 20)};  // TODO Decide how fast it should accelerate

        out.steerFF = {
                0.0_V, 0.0_V / 1_rad_per_s,
                0.0_V / 1_rad_per_s_sq};  // TODO Use Sysid to find constants
        out.wheelFF = wheelFFs[i];
        out.analogOffset = analogOffsets[i];
        out.pctMultiplier = pctMultiplier[i];
        moduleConfigs.emplace_back(out);
    }


    for (size_t i = 0; i < modules.size(); i++) {
        const auto prefix = motorPrefixes[i];
        auto steerName = fmt::format("{}_steer", prefix);
        auto wheelName = fmt::format("{}_wheel", prefix);
        auto steerMotor = motorHandler.getMotor(steerName);
        auto wheelMotor = motorHandler.getMotor(wheelName);
        modules[i] = std::make_unique<SwerveModule>(prefix, steerMotor, wheelMotor,
                                                    moduleConfigs[i]);
    }
//    steerPIDNT = std::make_unique<NetworkTablePID>(
//            "EctoSwerve/SteerPID", moduleConfig.steerPID, [&](const auto &config) {
//                for (auto &module: modules) {
//                    module->setSteerPID(config);
//                }
//            });
//
//    wheelPIDNT = std::make_unique<NetworkTablePID>(
//            "EctoSwerve/WheelPID", moduleConfig.wheelPID, [&](const auto &config) {
//                for (auto &module: modules) {
//                    module->setWheelPID(config);
//                }
//            });

    table = ntInstance.GetTable("EctoSwerve");

    xFilter.Reset();
    yFilter.Reset();

#ifndef SIMULATION

#ifndef USE_NAVX
    /**
     * Gyro init
     */
    adis = std::make_unique<frc::ADIS16470_IMU>(
            frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0,
            frc::ADIS16470_IMU::CalibrationTime::_4s);
    // adis.Calibrate();

    adis->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
#else
    navx = std::make_unique<AHRS>(frc::SerialPort::Port::kUSB, AHRS::SerialDataType::kProcessedData, 100); //TODO @gus check if this doesn't die
    navx->Calibrate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    double startTime = frc::Timer::GetFPGATimestamp().value();

    while (navx->IsCalibrating()) {
        double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
        if (timePassed > 16) {
            std::cout << "ERROR!!!!: NavX took too long to calibrate." << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#endif

#endif

    thetaController = std::make_shared<SwerveThetaController>();

    zeroYaw();
    initNetworkTables();
}

void EctoSwerve::initNetworkTables() { ; }

void EctoSwerve::updateNetworkTables() {
    table->GetEntry("CurrentHeading").SetDouble(getYaw());
    table->GetEntry("velMagnitude").SetDouble(velMagnitude);
    table->GetEntry("omega").SetDouble(getYawRate());
    auto currentPose = getPose();

    table->GetEntry("Odometry/Velocity/dX").SetDouble(currentVelocity.vx.value());
    table->GetEntry("Odometry/Velocity/dY").SetDouble(currentVelocity.vy.value());
    table->GetEntry("Odometry/Velocity/dTheta")
            .SetDouble(currentVelocity.omega.value());



    table->GetEntry("Odometry/X").SetDouble(currentPose.X().value());
    table->GetEntry("Odometry/Y").SetDouble(currentPose.Y().value());
    table->GetEntry("Odometry/Heading")
            .SetDouble(currentPose.Rotation().Radians().value());
    table->GetEntry("Odometry/Point")
            .SetString(fmt::format("({},{})", currentPose.X().value(),
                                   currentPose.Y().value()));
}

void EctoSwerve::setVelocity(const frc::ChassisSpeeds &target) {

    auto modTarget = target;

    if(thetaController->isEnabled()){
        modTarget.omega = units::radians_per_second_t(thetaController->getTarget().value() / 12.0);
        lastSetTime = frc::Timer::GetFPGATimestamp();
    }

    auto wpiStates = kinematics->ToSwerveModuleStates(modTarget);
    kinematics->DesaturateWheelSpeeds(&wpiStates, units::meters_per_second_t(4.65));

    auto states = SwerveState(wpiStates);
    SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState,
                                0);
    setModules(states, MotorControlMode::Velocity);
    lastState = states;
}

void EctoSwerve::setPercent(const frc::ChassisSpeeds &target,
                            const Point2D &point) {
    frc::Translation2d cor(units::meter_t(point.getX()),
                           units::meter_t(point.getY()));

    auto modTarget = target;

    if(thetaController->isEnabled()){
        modTarget.omega = units::radians_per_second_t(thetaController->getTarget().value() / 12.0);
        lastSetTime = frc::Timer::GetFPGATimestamp();
    }

    auto wpiStates = kinematics->ToSwerveModuleStates(modTarget, cor);
    kinematics->DesaturateWheelSpeeds(&wpiStates, units::meters_per_second_t(12));

    auto states = SwerveState(wpiStates);
    SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState,
                                0);
    setModules(states, MotorControlMode::Percent);
    lastState = states;
}

void EctoSwerve::setVoltage(const frc::ChassisSpeeds &target) {
    auto modTarget = target;

    if(thetaController->isEnabled()){
        modTarget.omega = units::radians_per_second_t(thetaController->getTarget().value());
        lastSetTime = frc::Timer::GetFPGATimestamp();
    }

    auto wpiStates = kinematics->ToSwerveModuleStates(modTarget);
    kinematics->DesaturateWheelSpeeds(&wpiStates,
                                      units::meters_per_second_t(12.0));
    auto states = SwerveState(wpiStates);
    SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState,
                                0);

    setModules(states, MotorControlMode::Voltage);
    lastState = states;
}

SwerveState EctoSwerve::getMotorStates() const {
    SwerveState values;

    for (int i = 0; i < 4; i++) {
        *values.wheels[i] = modules[i]->getState();
    }

    return values;
}

void EctoSwerve::setModules(const SwerveState &rawSetpoint,
                            MotorControlMode controlMode) {
    auto state = getMotorStates();
    auto dt = frc::Timer::GetFPGATimestamp().value() - lastRunTime;

    auto setpoint = SwerveState::optimizeValues(state, rawSetpoint, lastState,
                                                lastSetpoint, dt);

    for (int i = 0; i < 4; i++) {
        auto wheel = setpoint.wheels[i];
        modules[i]->setState(wheel->wheelAngle, wheel->wheelVelocity, controlMode);
    }

    lastSetpoint = setpoint;
    lastState = state;
    lastRunTime = frc::Timer::GetFPGATimestamp().value();
}

void EctoSwerve::breakWithX() {
//    std::vector<frc::Rotation2d> defaultAngles = {{-45_deg},{45_deg}, {45_deg}, {-45_deg}}; // o mode
    std::vector<frc::Rotation2d> defaultAngles = {{45_deg},{-45_deg}, {-45_deg}, {45_deg}}; // X mode
    std::array<frc::SwerveModuleState, 4> states;
    for (size_t i = 0; i < states.size() ; i++){
        states[i].angle = defaultAngles[i];
        states[i].speed = 0_mps;
    }
    setModules({states}, MotorControlMode::Percent);
}

void EctoSwerve::brakeWithH() {
//    std::vector<frc::Rotation2d> defaultAngles = {{-45_deg},{45_deg}, {45_deg}, {-45_deg}}; // o mode
    std::vector<frc::Rotation2d> defaultAngles = {{0_deg},{0_deg}, {0_deg}, {0_deg}}; // X mode
    std::array<frc::SwerveModuleState, 4> states;
    for (size_t i = 0; i < states.size() ; i++){
        states[i].angle = defaultAngles[i];
        states[i].speed = 0_mps;
    }

    for (int i = 0; i < 4; i++) {
        modules[i]->setState(0.0, 0.0, MotorControlMode::Percent);
    }

//    setModules({states}, MotorControlMode::Percent);
}

void EctoSwerve::robotInit() {
    ;
}

void EctoSwerve::robotUpdate() {
    updateNetworkTables();
    auto motorStates = getMotorStates();
    updateOdometry(motorStates);
    currentVelocity = kinematics->ToChassisSpeeds(motorStates.toWPIVel());

    velMagnitude = frc::Velocity2d(currentVelocity.vx, currentVelocity.vy).Norm().value();

    double currentTime = frc::Timer::GetFPGATimestamp().value();
    double dt = currentTime - prevTimeStamp;
    double dv = velMagnitude - prevVelMag;
    double accelMag = dv / dt;
    if (accelMag > 0.5) {
        std::cout << "more than 0.5 accleration, aaaaaaaaa" << std::endl;
    }


    /**
     * Only theta controller is being used, call set voltage with 0 vx,vy
     * so that theta can be set
     */
    if(thetaController->isEnabled() and frc::Timer::GetFPGATimestamp() - lastSetTime > 25_ms){
        frc::ChassisSpeeds vel;
        setVoltage(vel);
    }

    if(thetaController->isEnabled() != lastThetaControllerState and !thetaController->isEnabled()){
        //Manually set velocity to 0 after theta controller is disabled
        std::cout << "Theta controller disabled" << std::endl;
        frc::ChassisSpeeds vel;
        setVoltage(vel);
    }

    lastThetaControllerState = thetaController->isEnabled();

    auto state = getPose();
    if(std::isnan(state.X().value()) or std::isnan(state.Y().value())){
        log->error("Odometry nan, resetting position!");
        pe_NMutex.lock();
        pe_MMutex.lock();
        pe_NMutex.unlock();
        estimator->ResetPosition(
                {},
//                getMotorStates().toWPIPose(),
                {});
        pe_MMutex.unlock();
    }
}

void EctoSwerve::zeroYaw() {
    /**
     * NavX Reset
     */
    headingZero = getYaw(false);
    rollZero = getRoll(false);
    pitchZero = getPitch(false);
}

void EctoSwerve::setYaw(double yaw) {
#ifndef SIMULATION
    headingZero = getYaw(false) - yaw;
#else
    simYaw = yaw;
#endif
}

double EctoSwerve::getRawYaw() const {
#ifdef USE_NAVX
#ifndef SIMULATION
    return -navx->GetAngle() * (M_PI) / 180.0;
#endif
#endif
    return 0; //TODO
}

double EctoSwerve::getYaw(bool useZero) const {
    double yaw = 0;

#ifndef SIMULATION
#ifndef USE_NAVX
    yaw = adis->GetAngle().value() * (M_PI / 180.0);
    yaw = EctoMath::wrapAngle(yaw);
#else
    yaw = -navx->GetYaw() * (M_PI / 180.0);
    yaw = EctoMath::wrapAngle(yaw);
#endif

#else
    yaw = EctoMath::wrapAngle(simYaw);
#endif
    if (useZero) { yaw -= headingZero; }
    yaw = EctoMath::wrapAngle(yaw);
    return yaw;
}

double EctoSwerve::getRoll(bool useZero) const {
    double roll = 0;
#ifndef SIMULATION

#ifndef USE_NAVX
    //roll = adis->GetYawAxis(ADIS16470_IMU::IMUAxis::kX);

#else
    roll = navx->GetRoll() * (M_PI / 180.0);
#endif

#endif

    roll = EctoMath::wrapAngle(roll);
    if (useZero) { roll -= rollZero; }
    roll = EctoMath::wrapAngle(roll);
    return roll;
}

double EctoSwerve::getPitch(bool useZero) const {
    double pitch = 0;
#ifndef SIMULATION
#ifndef USE_NAVX
    //pitch = adis->GetYawAxis(ADIS16470_IMU::IMUAxis::kY);

#else
    pitch = navx->GetPitch() * (M_PI / 180.0); //navx2 is inverted i dont understand what the hell, hmmmmmmmmmmm.
#endif
#endif

    pitch = EctoMath::wrapAngle(pitch);
    if (useZero) { pitch -= pitchZero; }
    pitch = EctoMath::wrapAngle(pitch);
    return pitch;
}

frc::Pose2d EctoSwerve::getPose() {
    pe_NMutex.lock();
    std::lock_guard<std::mutex> lg(pe_MMutex);
    pe_NMutex.unlock();

    auto pose = estimator->GetEstimatedPosition();
//
//    auto x = units::meter_t(xFilter.Calculate(pose.X().value()));
//    auto y = units::meter_t(yFilter.Calculate(pose.Y().value()));
//
//
//    return {x, y, pose.Rotation()};
    return pose;
}

void EctoSwerve::addVisionPoseMeasurement(const frc::Pose2d visionPose, double timeStamp) {
    //Low priority lock
    pe_LMutex.lock();
    pe_NMutex.lock();
    pe_MMutex.lock();
    pe_NMutex.unlock();
    estimator->AddVisionMeasurement(visionPose, units::second_t(timeStamp));
    pe_MMutex.unlock();
    pe_LMutex.unlock();
}


void EctoSwerve::resetOdometry(const frc::Pose2d newPose) {
    setYaw(newPose.Rotation().Radians().value());
    pe_NMutex.lock();
    pe_MMutex.lock();
    pe_NMutex.unlock();
    estimator->ResetPosition(
            newPose,
//            {getMotorStates().toWPIPose()},
            newPose.Rotation());  // TODO Maybe use odometry to return the heading, it

    // can handle setting the heading on its own
    pe_MMutex.unlock();
}

frc::Velocity2d EctoSwerve::getVelocity() const {
    return {currentVelocity.vx, currentVelocity.vy};
}

frc::Rotation2d EctoSwerve::getRotation()  {
    return getPose().Rotation();
}

frc::ChassisSpeeds EctoSwerve::getChassisSpeeds(){
    return currentVelocity;
}

void EctoSwerve::updateOdometry(const SwerveState &motorStates) {
    auto moduleStates = motorStates.toWPIVel();
    auto modulePoses = motorStates.toWPIPose();
//    currentVelocity = kinematics->ToChassisSpeeds(moduleStates);

    pe_NMutex.lock();
    pe_MMutex.lock();
    pe_NMutex.unlock();
    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp() ,Rotation2d(units::angle::radian_t(getYaw())),
                     moduleStates, modulePoses);
    pe_MMutex.unlock();
}

std::vector<std::shared_ptr<ChecklistItem>> EctoSwerve::createTests() {
    return {};
}

units::volt_t EctoSwerve::calculateRotationFF(const units::radians_per_second_t &velocity) const {
    return rotationFF.Calculate(velocity);
}