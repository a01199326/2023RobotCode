//
// Created by cc on 17/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_SETJOINT_H
#define BOTBUSTERS_REBIRTH_SETJOINT_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"

class SetJoint : public frc2::CommandHelper<frc2::CommandBase, SetJoint>{
public:
    explicit SetJoint(const std::shared_ptr<PIDJoint> &joint,
                      units::radian_t setAngle
                      );

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    std::shared_ptr<PIDJoint> joint;
    std::shared_ptr<PIDTelescopic> telescopic;
    units::radian_t setAngle;
};


#endif //BOTBUSTERS_REBIRTH_SETJOINT_H
