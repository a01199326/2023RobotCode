//
// Created by cc on 23/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_SETWRIST_H
#define BOTBUSTERS_REBIRTH_SETWRIST_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/PID/PIDWrist/PIDWrist.h"

class SetWrist : public frc2::CommandHelper<frc2::CommandBase, SetWrist>{
public:
    explicit SetWrist(const std::shared_ptr<PIDWrist> &wrist,
                           units::radian_t setAngle,
                           bool endInstantly);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    std::shared_ptr<PIDWrist> wrist;
    units::radian_t setAngle;
    bool endInstantly;
};


#endif //BOTBUSTERS_REBIRTH_SETWRIST_H
