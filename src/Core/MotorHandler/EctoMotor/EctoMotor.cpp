//
// Created by hiram on 9/08/19.
//

#include <stdexcept>
#include <cmath>
#include <chrono>
#include "EctoMotor.h"

std::shared_ptr<spdlog::logger> EctoMotor::log = spdlog::stdout_color_mt("EctoMotor");

EctoMotor::EctoMotor(int motorID, const std::string &motorName, EctoMotorType motorType, bool doTest) {
	this->id = motorID;
	this->name = motorName;
	this->motorType = motorType;
    this->runBasicTests = doTest;
	
	log->trace("Creating motor with name {} and id {}...", motorName, motorID);
}

void EctoMotor::set(double value) {
	this->set(value, this->controlMode);
}

void EctoMotor::set(double value, MotorControlMode newControlMode) {
    bool isnanValue = std::isnan(value);
    if (isnanValue || std::isinf(value)) {
        value = 0.0;
        log->error(fmt::format("value set to motor: {}, id: {} is {}, set to zero for safety", name, id, isnanValue ? "nan" : "inf"));
    }
	
	controlMode = newControlMode;
	lastSetpoint = std::make_pair(newControlMode, value);
	
	switch (controlMode) {
		case MotorControlMode::Percent:
			this->setOutputPercent(value);
			break;
		case MotorControlMode::Velocity:
			this->setVelocitySetpoint(value);
			break;
		case MotorControlMode::Position:
			this->setPositionSetpoint(value);
			break;
		case MotorControlMode::MotionMagic:
			this->setMotionMagicOutput(value);
			break;
		case MotorControlMode::Current:
			this->setMotorOutputByCurrent(value);
			break;
		case MotorControlMode::Voltage:
			this->setVoltageOutput(value);
			break;
		default:
			throw std::logic_error("Invalid MotorControlMode");
	}
}

std::string EctoMotor::getName() const {
	return name;
}

void EctoMotor::setControlMode(MotorControlMode controlMode) {
	this->controlMode = controlMode;
}

MotorControlMode EctoMotor::getControlMode() const {
	return this->controlMode;
}

void EctoMotor::setFeedbackMode(MotorFeedbackMode feedbackMode) {
	this->feedbackMode = feedbackMode;
	switch (this->feedbackMode) {
		case MotorFeedbackMode::None:
			throw std::runtime_error("Cant set feedbackmode to None in motor with name: " + name);
		case MotorFeedbackMode::QuadEncoder:
			this->setQuadAsClosedLoopSource();
			break;
		case MotorFeedbackMode::Potentiometer:
			this->setPotAsClosedLoopSource();
			break;
        case MotorFeedbackMode::AbsoluteEncoder:
            this->setAbsoluteAsClosedLoopSource();
	}
}

MotorFeedbackMode EctoMotor::getFeedbackMode() const {
	return this->feedbackMode;
}

int EctoMotor::getId() const {
	return id;
}

bool EctoMotor::isDisabled() const {
	return disabled;
}

double EctoMotor::getPosition() const {
	switch (feedbackMode) {
		case MotorFeedbackMode::None:
			log->warn("Cannot get a position when no feedback mode is defined in motor with name: {} ", name);
			return 0;
		case MotorFeedbackMode::QuadEncoder:
			return getQuadPosition();
		case MotorFeedbackMode::Potentiometer:
			return getPotPosition();
        case MotorFeedbackMode::AbsoluteEncoder:
            return getAbsPosition();
		default:
			return 0;
	}
}

double EctoMotor::getVelocity() const {
	switch (feedbackMode) {
		case MotorFeedbackMode::None:
			log->warn("Cannot get a velocity when no feedback mode is defined in motor with name: {} ", name);
			return 0;
		case MotorFeedbackMode::QuadEncoder:
			return getQuadVelocity();
		case MotorFeedbackMode::Potentiometer:
			return getPotVelocity();
        case MotorFeedbackMode::AbsoluteEncoder:
            return getAbsVelocity();
		default:
			return 0;
	}
}

double EctoMotor::getMotorEncoderPos() const {
    return getQuadPosition();
}

double EctoMotor::getMotorEncoderVel() const {
    return getQuadVelocity();
}

EctoMotorType EctoMotor::getMotorType() const {
	return motorType;
}

std::pair<MotorControlMode, double> EctoMotor::getLastSetpoint() const {
	return lastSetpoint;
}

void EctoMotor::outputSet(double set) {
	this->set(set, MotorControlMode::Voltage);
}
