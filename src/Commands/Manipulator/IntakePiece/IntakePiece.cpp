//
// Created by cc on 12/02/23.
//

#include "IntakePiece.h"
#include "Commands/Utilities/WaitForButton/WaitForButton.h"


IntakePiece::IntakePiece(
        const std::shared_ptr<PIDJoint> &joint,
        const std::shared_ptr<PIDTelescopic> &telescopic,
        const std::shared_ptr<PIDWrist> &wrist,
        const std::shared_ptr<Manipulator> &manipulator,
        const std::shared_ptr<VisionManager> &visionManager,
                         const State &state,
                         const ManipulatorStates &states,
                         ScorePiece scorePiece,
                         bool useHumanInput) {


    bool isCube = scorePiece == ScorePiece::CUBE;

    double speed = state.manipulatorMotor;

    units::second_t time = isCube ? 0.2_s : 0.05_s;


    frc2::SequentialCommandGroup::AddCommands(
            frc2::PrintCommand(fmt::format("isCube: {}", isCube)),
            SetManipulator(manipulator, speed, true, true),
            frc2::InstantCommand([wrist]{auto config = wrist->getConfig(); wrist->setConstraints(config.intakeMaxVel, config.intakeMaxAccel);}),
            SetRobotState(joint, wrist, telescopic, state),
            frc2::InstantCommand([wrist]{wrist->setDefaultConstraints();}),
            frc2::ConditionalCommand(
                    WaitForButton("rightBumper", 0),
                    frc2::WaitUntilCommand([manipulator = manipulator] {return manipulator->hasPiece();}),
                    {[useHumanInput = useHumanInput] {return useHumanInput;}}
            ),
            frc2::PrintCommand(fmt::format("time: {}", time)),
            frc2::WaitCommand(time),
            SetManipulator(manipulator, speed / 1.0001 , false, true),
            SetRobotState(joint, wrist, telescopic, states.idle),
            SetManipulator(manipulator, -0.065, false, true)
    );
}