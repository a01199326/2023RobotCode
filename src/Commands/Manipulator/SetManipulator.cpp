//
// Created by cc on 23/01/23.
//

#include "SetManipulator.h"
#include <iostream>

SetManipulator::SetManipulator(const std::shared_ptr<Manipulator> &manipulator,
                               double setMotor,
                               bool opened,
                               bool endInstantly,
                               MotorControlMode controlMode) {
    this->manipulator = manipulator;
    this->setMotor = setMotor;
    this->opened = opened;
    this->controlMode = controlMode;
    this->endInstantly = endInstantly;

    AddRequirements({manipulator.get()});
}

void SetManipulator::Initialize() { ; }

void SetManipulator::Execute() {
    manipulator->set(setMotor, controlMode);
}

bool SetManipulator::IsFinished() {
    return endInstantly;
}

void SetManipulator::End(bool interrupted) {
    if (!endInstantly) {
        manipulator->idle();
    }
}