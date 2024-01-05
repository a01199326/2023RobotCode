//
// Created by cc on 23/11/21.
//

#include "IsOnTarget.h"
#include <frc/geometry/Pose2d.h>
#include <iostream>


IsOnTarget::IsOnTarget(const std::shared_ptr<EctoSwerve> &swerve, const frc::Translation2d &point, units::meter_t tol) {
    this->point = point;
    this->tol = tol;
    this->swerve = swerve;
}

void IsOnTarget::Initialize() { ; }

void IsOnTarget::Execute() { ; }

void IsOnTarget::End(bool interrupted) {
    std::cout << "is in zone" << std::endl;
}


bool IsOnTarget::IsFinished() {
	auto pose = swerve->getPose();
	auto dist = point.Distance(pose.Translation());
	return dist < tol;
}

