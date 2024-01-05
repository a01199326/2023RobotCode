//
// Created by cc on 11/02/23.
//

#include "ManipulateManipulator.h"


ManipulateManipulator::ManipulateManipulator(const std::shared_ptr<PIDJoint> &joint,
                                             const std::shared_ptr<PIDWrist> &wrist,
                                             const std::shared_ptr<PIDTelescopic> &telescopic,
                                             const std::shared_ptr<Manipulator> &manipulator,
                                             const std::shared_ptr<PWMColorSensor> &colorSensor,
                                             const ManipulatorStates &states,
                                             ScoreHeight height, ScorePiece scorePiece, bool waitForInput) {
    State state;
    units::second_t time;
    if (scorePiece == ScorePiece::CONE) {
        if (height == ScoreHeight::HIGH) state = states.teleopScoreConeHight;
        if (height == ScoreHeight::MID) state = states.teleopScoreConeMid;
        if (height == ScoreHeight::LOW) state = states.scoreConeLow;
        time = 0.1_s;
    } else if (scorePiece == ScorePiece::CUBE) {
        if (height == ScoreHeight::HIGH) state = states.teleopScoreCubeHigh;
        if (height == ScoreHeight::MID) state = states.scoreCubeMid;
        if (height == ScoreHeight::LOW) state = states.scoreCubeLow;
        time = 0.1_s;
    } else{
        if (height == ScoreHeight::HIGH) state = states.scoreCubeHigh;
        if (height == ScoreHeight::MID) state = states.scoreCubeMid;
        if (height == ScoreHeight::LOW) state = states.scoreCubeLow;
        time = 0.15_s;

    }

    frc2::SequentialCommandGroup::AddCommands(
            SetRobotState(joint, wrist, telescopic, state),
            frc2::ConditionalCommand(
                    WaitForButton("rightBumper", 0),
                    frc2::WaitCommand(0.25_s),
                    {[waitForInput = waitForInput] {return waitForInput;}}
            ),
            frc2::ParallelDeadlineGroup(
                    frc2::SequentialCommandGroup(
                            frc2::WaitCommand(time),
                            SetRobotState(joint, wrist, telescopic, states.idle, false)
                    ),
                    SetManipulator(manipulator, state.manipulatorMotor, state.manipulatorPiston, false)
            ),
            SetManipulator(manipulator, -0.1, false, true)


    );
}