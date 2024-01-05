#include "SwerveModule.h"

#include <utility>
#include "Math/EctoMath.h"
#include <frc/Timer.h>
#include <frc/DriverStation.h>

SwerveModule::SwerveModule(const std::string &name, const std::shared_ptr<EctoMotor> &steerMotor,
                           const std::shared_ptr<EctoMotor> &wheelMotor, const SwerveModuleConfig &config) {
	moduleName = name;
	
	this->steerMotor = steerMotor;
	this->wheelMotor = wheelMotor;

	steerMotor->enableBrakingOnIdle(true);
	wheelMotor->enableBrakingOnIdle(true);
	
	steerMotor->setMotorCurrentLimit(config.steerCurrentLimit);
	steerMotor->enableCurrentLimit(true);
	
	wheelMotor->setMotorCurrentLimit(config.wheelCurrentLimit);
	wheelMotor->enableCurrentLimit(true);
	
	steerMotor->setAnalogPositionConversionFactor(2 * M_PI / 3.33333333); //Potentiometer v/rot
    steerMotor->setAnalogVelocityConversionFactor(2 * M_PI / 3.33333333);
    steerMotor->setAnalogSensorOffset(config.analogOffset);
	analogOffset = config.analogOffset;
	
	steerMotor->setFeedbackMode(MotorFeedbackMode::Potentiometer);
    steerMotor->setOpenLoopRampRate(0.0001);
    steerMotor->setClosedLoopRampRate(0.0001);
	
	wheelMotor->invertMotor(false);
	steerMotor->invertMotor(false);
	steerMotor->invertSensor(true);
	
	wheelMotor->setClosedLoopRampRate(0.115);
	wheelMotor->setOpenLoopRampRate(0.115);

    steerMotor->enableLimitSwitches(false);
    steerMotor->enableReverseSoftLimit(false);
    steerMotor->enableForwardSoftLimit(false);

    wheelMotor->enableLimitSwitches(false);
    wheelMotor->enableReverseSoftLimit(false);
    wheelMotor->enableForwardSoftLimit(false);

    steerMotor->setPIDConfig(config.steerPID);
    steerMotor->enableContinuousInput(-M_PI, M_PI);

    wheelMotor->setPIDConfig(config.wheelPID);

    steerMotor->burnFlash();
    wheelMotor->burnFlash();
	
	wheelCircumference = config.wheelCircumference;
	gearRatio = config.gearRatio;

	table = nt::NetworkTableInstance::GetDefault().GetTable(fmt::format("SwerveModule/{}", moduleName));
	velocityEntry = table->GetEntry("Velocity");
	angleEntry = table->GetEntry("Angle");
	wheelSetpointEntry = table->GetEntry("WheelSetpoint");
	steerSetpointEntry = table->GetEntry("SteerSetpoint");
    wheelPositionEntry = table->GetEntry("wheelDistance");
	controlModeEntry = table->GetEntry("ControlMode");
	encoderState = table->GetEntry("EncoderPosition");
	rawAnalog = table->GetEntry("RawAnalog");
    ffEntry = table->GetEntry("FF");
//    steerCurrentEntry = table->GetEntry("SteerCurrent");
    wheelCurrentEntry = table->GetEntry("WheelCurrent");
//    steerErrorEntry = table->GetEntry("steerError");
//    velErrorEntry = table->GetEntry("velError");
//    driveTempEntry = table->GetEntry("driveTemp");
//    steerTempEntry = table->GetEntry("steerTemp");

	
	ntNotifier = std::make_unique<frc::Notifier>([this] {
		this->updateNT();
	});
	
	 ntNotifier->StartPeriodic(ntUpdateRate);
     pctMultiplier = config.pctMultiplier;
}

void SwerveModule::setState(double angle, double motorSetpoint, MotorControlMode wheelControlMode) {
    std::lock_guard<std::mutex> lg(stateMutex);

//    motorSetpoint *= std::cos(steerController->GetPositionError());
    steerMotor->set(angle, MotorControlMode::Position);

    if (wheelControlMode == MotorControlMode::Percent) {
        motorSetpoint *= pctMultiplier;
    }

	if(wheelControlMode == MotorControlMode::Velocity){
        motorSetpoint = (motorSetpoint / wheelCircumference) * gearRatio * 2 * M_PI;
	}


    wheelMotor->set(motorSetpoint, wheelControlMode);

    controlMode = wheelControlMode;
	wheelSetpoint = motorSetpoint;
	steerSetpoint = angle;
}

SwerveWheel SwerveModule::getState() const {
    std::lock_guard<std::mutex> lg(stateMutex);
    SwerveWheel out;

    out.wheelPosition = (wheelMotor->getPosition() / (2.0 * M_PI)) / gearRatio * wheelCircumference;

    units::second_t currentTime = frc::Timer::GetFPGATimestamp();
    units::second_t dt = prevTime >= 0_s ? currentTime - prevTime : 0.02_s;
    prevTime = currentTime;

    if(lastStateUpdate == 0_s){
        //Initial state, velocity is 0
        out.wheelVelocity = 0;
    } else{
        out.wheelVelocity = (out.wheelPosition - lastWheelPosition) / dt.value();
    }

    if(!usePosDeltaVelocity){
        out.wheelVelocity = (wheelMotor->getVelocity() / (2.0 * M_PI)) / gearRatio * wheelCircumference;
//        out.wheelVelocity = (out.wheelPosition - lastWheelPosition) / dt.value();
    }


    lastStateUpdate = frc::Timer::GetFPGATimestamp();
	out.wheelAngle = steerMotor->getPosition();
	out.wheelAngularVelocity = steerMotor->getVelocity();
	lastWheelPosition = out.wheelPosition;

	return out;
}

void SwerveModule::updateNT() {
	auto state = getState();
	
	velocityEntry.SetDouble(state.wheelVelocity);
	angleEntry.SetDouble(state.wheelAngle);
	wheelSetpointEntry.SetDouble(wheelSetpoint);
	steerSetpointEntry.SetDouble(steerSetpoint);
	controlModeEntry.SetString(toString(controlMode));
	encoderState.SetDouble(wheelMotor->getPosition());
//    steerCurrentEntry.SetDouble(steerMotor->getCurrent());
    wheelCurrentEntry.SetDouble(wheelMotor->getCurrent());
//    steerErrorEntry.SetDouble(EctoMath::wrapAngle(steerSetpoint - steerMotor->getPosition()));
//    velErrorEntry.SetDouble(wheelSetpoint - (wheelMotor->getVelocity() / (2.0 * M_PI)) / gearRatio * wheelCircumference);

    wheelPositionEntry.SetDouble((wheelMotor->getPosition() / (2.0 * M_PI)) / gearRatio * wheelCircumference);
//    driveTempEntry.SetDouble(wheelMotor->getTemperature());
//    steerTempEntry.SetDouble(steerMotor->getTemperature());
	
	auto analogPos = steerMotor->getPosition();
	analogPos += analogOffset;
	analogPos = EctoMath::wrapAngle(analogPos);
	rawAnalog.SetDouble(analogPos);
}

