//
// Created by cc on 23/11/21.
//

#ifndef BOTBUSTERS_REBIRTH_ISONORNWARTARGET_CPP_H
#define BOTBUSTERS_REBIRTH_ISONORNWARTARGET_CPP_H

#include   "Systems/EctoSwerve/EctoSwerve.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <functional>
#include <functional>
#include <vector>
#include <cmath>

class IsOnTarget : public frc2::CommandHelper<frc2::CommandBase, IsOnTarget> {
public:
	IsOnTarget(const std::shared_ptr<EctoSwerve> &swerve, const frc::Translation2d &point, units::meter_t tol = 0.3_m);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	frc::Translation2d point;
	units::meter_t tol;
	bool check;
};

#endif //BOTBUSTERS_REBIRTH_ISONORNWARTARGET_CPP_H
