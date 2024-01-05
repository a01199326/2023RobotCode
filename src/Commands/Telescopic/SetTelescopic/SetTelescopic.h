//
// Created by cc on 21/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_SETTELESCOPIC_H
#define BOTBUSTERS_REBIRTH_SETTELESCOPIC_H

#include "Core/EctoModule/WPISubsystem.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Control/Kinematics/Arm/SimpleArmKinematics.h"
#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"


class SetTelescopic : public frc2::CommandHelper<frc2::CommandBase, SetTelescopic> {
public:
    explicit SetTelescopic(const std::shared_ptr<PIDTelescopic> &telescopic,
                           const std::shared_ptr<PIDJoint> &joint,
                           double setLength,
                           bool endInstantly = false);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    std::shared_ptr<PIDTelescopic> telescopic;
    std::shared_ptr<PIDJoint> joint;
    double setLength;
    bool endInstantly;

};


#endif //BOTBUSTERS_REBIRTH_SETTELESCOPIC_H
