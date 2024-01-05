//
// Created by cc on 23/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_SETMANIPULATOR_H
#define BOTBUSTERS_REBIRTH_SETMANIPULATOR_H


#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Systems/Generic/Manipulator/Manipulator.h"

class SetManipulator : public frc2::CommandHelper<frc2::CommandBase, SetManipulator>{
public:
    explicit SetManipulator(const std::shared_ptr<Manipulator> &manipulator,
                            double setMotor,
                            bool opened,
                            bool endInstantly = false,
                            MotorControlMode controlMode = MotorControlMode::Percent);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    std::shared_ptr<Manipulator> manipulator;
    double setMotor;
    MotorControlMode controlMode;
    bool opened, endInstantly;
};


#endif //BOTBUSTERS_REBIRTH_SETMANIPULATOR_H
