//
// Created by cc on 23/01/23.
//

#include "SetJoint.h"
#include <iostream>


SetJoint::SetJoint(const std::shared_ptr<PIDJoint> &joint,
                   units::radian_t setAngle
                   ) {
    this->joint = joint;
    this->setAngle = setAngle;
    AddRequirements({joint.get()});
}

void SetJoint::Initialize() {
    double diff = std::abs(setAngle.value() - joint->getAngle().value());
    if (diff > 1.0){
        joint->setConstraints(joint->getMaxVel(), joint->getMaxAccel());
    }else {
        joint->setConstraints(joint->getMaxVel(), joint->getMaxAccel());
    }
}

void SetJoint::Execute() {
    joint->set(setAngle);
}

bool SetJoint::IsFinished() {
    return joint->inTol(0.09_rad);
}

void SetJoint::End(bool interrupted) {
    std::cout << "arc good" << std::endl;
}
