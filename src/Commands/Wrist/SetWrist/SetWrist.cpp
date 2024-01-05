//
// Created by cc on 23/01/23.
//

#include "SetWrist.h"
#include <iostream>


SetWrist::SetWrist(const std::shared_ptr<PIDWrist> &wrist,
                   units::radian_t setAngle,
                   bool endInstantly) {
    this->wrist = wrist;
    this->setAngle = setAngle;
    this->endInstantly = endInstantly;
    AddRequirements({wrist.get()});
}

void SetWrist::Initialize() { ; }

void SetWrist::Execute() {
    wrist->set(setAngle);
}

bool SetWrist::IsFinished() {
    return endInstantly || wrist->inTol(0.3_rad);
}

void SetWrist::End(bool interrupted) {
    std::cout << "wrist good" << std::endl;
}
