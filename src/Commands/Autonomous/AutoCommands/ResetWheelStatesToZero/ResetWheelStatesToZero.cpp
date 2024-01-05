//
// Created by cc on 06/01/22.
//

#include "ResetWheelStatesToZero.h"

ResetWheelStateToZero::ResetWheelStateToZero(const std::shared_ptr<EctoSwerve> &swerve, bool endInstantly) {
	this->swerve = swerve;
    this->endInstantly = endInstantly;
}

void ResetWheelStateToZero::Initialize() {
	;
}

void ResetWheelStateToZero::Execute() {
	std::array<frc::SwerveModuleState, 4> states;
	swerve->setModules(states);
}

void ResetWheelStateToZero::End(bool interrupted) {
	;
}

bool ResetWheelStateToZero::IsFinished() {
	return endInstantly;
}