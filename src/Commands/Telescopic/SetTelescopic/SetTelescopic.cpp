//
// Created by cc on 21/01/23.
//

#include "SetTelescopic.h"
#include <iostream>
#include <frc/DriverStation.h>

SetTelescopic::SetTelescopic(const std::shared_ptr<PIDTelescopic> &telescopic,
                             const std::shared_ptr<PIDJoint> &joint,
                             double setLength,
                             bool endInstantly) {
    this->telescopic = telescopic;
    this->joint = joint;
    this->setLength = setLength;
    this->endInstantly = endInstantly;

    AddRequirements({telescopic.get()});

}

void SetTelescopic::Initialize() {
//    double diff = joint->getGoal().value() - joint->getAngle().value();
//    if (diff > 0.0){
    auto config = telescopic->getConfig();
    auto upVel = config.maxGoingUpTeleopVel;
    auto upAccel = config.maxGoingUpTeleopAccel;
    auto downVel = config.maxGoingDownTeleopVel;
    auto downAccel = config.maxGoingDownTeleopAccel;

    if (frc::DriverStation::IsAutonomous()) {
        upVel = config.maxGoingUpAutoVel;
        upAccel = config.maxGoingUpAutoAccel;
        downVel = config.maxGoingDownAutoVel;
        downAccel = config.maxGoingDownAutoAccel;

    }

    if ((setLength - telescopic->getHeight().value()) > 0){
        telescopic->setConstraints(upVel, upAccel);//going up
    }else{
        telescopic->setConstraints(downVel, downAccel);//going down
    }
//    }else {
//        telescopic->setConstraints(telescopic->getMaxVel(), telescopic->getMaxAccel());
//    }
}

void SetTelescopic::Execute() {
    telescopic->set(setLength);
}

bool SetTelescopic::IsFinished() {
    return endInstantly || telescopic->inTol(9_cm);
}

void SetTelescopic::End(bool interrupted) {
    std::cout << "telescopic good" << std::endl;
}
