//
// Created by cc on 11/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_MANIPULATORSTATE_H
#define BOTBUSTERS_REBIRTH_MANIPULATORSTATE_H

#include "Core/Robot/RobotState.h"

struct ManipulatorStates {
    State idle = {}; //TODO

    State intakeConeUpRight = {};
    State autoIntakeConeUpRight = {};
    State intakeConeFloor = {};
    State intakeCubeFloor = {};

    State intakeStation = {};
    State intakeDoubleStation = {};
    State intakeCubeAuto = {};

    State tiltStation = {};
//    State tiltStationRed = {};
    State idleStation = {};

    State scoreConeLow = {};
    State scoreConeMid = {};
    State teleopScoreConeMid = {};
    State autoScoreConeMid = {};
    State scoreConeHigh = {};
    State teleopScoreConeHight = {};

    State scoreCubeLow = {};
    State scoreCubeMid = {};
    State autoScoreCubeMid = {};
    State scoreCubeHigh = {};
    State teleopScoreCubeHigh = {};

    State balancePosition = {};

    static const ManipulatorStates back() {
        ManipulatorStates states;
        states.idle = State(-0.257_rad, -1.76_rad, 0.03_m);

        states.intakeConeUpRight = State(-1.38_rad, 0.92_rad, 0.03_m, false, -1.0);
        states.autoIntakeConeUpRight = State(-1.371_rad, 0.92_rad, 0.03_m, false, -1.0);
        states.intakeConeFloor = State(-1.754_rad, 0.255_rad, 0.03_m, false, -0.5);
        states.intakeCubeFloor = State(-1.55_rad, 0.713_rad, 0.03_m, false, -1.0);
        states.intakeCubeAuto = State(-1.549_rad, 0.71_rad, 0.03_m, false, -1.0);


        states.intakeStation = State(-1.457_rad, -0.73_rad, 0.03_m, false, -0.85); //TODO
        states.intakeDoubleStation = State(-0.33_rad, 1.086_rad, 0.03_m, false, -0.7);
//        states.intakeDoubleStation = State(-0.23_rad, 1.92_rad, 0.365_m, false, -0.85);

        states.tiltStation = State(-1.583_rad, 1.4_rad, 0.035_m);
//        states.tiltStationRed = State(-1.571_rad, 1.4_rad, 0.035_m);
//        states.idleStation = State(-1.65_rad, 1.5_rad, 0.005_m);

        states.scoreConeLow = State(-1.86_rad, 0.37_rad, 0.03_m);
        states.scoreConeMid = State(-0.63_rad, 1.555_rad, 0.351_m, true, 0.52); //0.22m the before
        states.teleopScoreConeMid = State(-0.63_rad, 1.555_rad, 0.451_m, true, 0.52);
        states.autoScoreConeMid = State(-0.63_rad, 1.52_rad, 0.336_m, true, 0.52);
        states.scoreConeHigh = State(-0.68_rad, 71_deg, 1.04_m, true, 0.52);
        states.teleopScoreConeHight = State(-0.68_rad, 71_deg, 0.99_m, true, 0.52);

        states.scoreCubeLow = State(-1.86_rad, 0.37_rad, 0.03_m);
        states.scoreCubeMid = State(-0.257_rad, 1.5_rad, 0.03_m, true, 0.5);
        states.autoScoreCubeMid = State(-0.257_rad, 1.45_rad, 0.03_m, true, 0.5);
        states.scoreCubeHigh = State(-0.72_rad, 0.86_rad, 0.4_m, true, 0.5);
        states.teleopScoreCubeHigh = State(-0.667_rad, 0.86_rad, 0.47_m, true, 0.5);

        states.balancePosition = State(-1.65_rad, 1.5_rad, 0.03_m, false, 0.0);

        return states;
    }

    static const ManipulatorStates front() {
        ManipulatorStates states;
        states.idle  = State(0.255_rad, 1.76_rad, 0.01_m);

        states.intakeConeUpRight = State(1.38_rad, -0.92_rad, 0.01_m, false, -1.0);
        states.autoIntakeConeUpRight = State(1.38_rad, -0.92_rad, 0.01_m, false, -1.0);
        states.intakeConeFloor = State(1.63_rad, -0.875_rad, 0.01_m, false, -0.5);
        states.intakeCubeFloor = State(1.55_rad, -0.74_rad, 0.01_m, false, -1.0);
        states.intakeCubeAuto = State(1.549_rad, -0.693_rad, 0.01_m, false, -1.0);

        states.intakeStation = State(1.457_rad, 0.73_rad, 0.01_m, false, -0.85); //TODO
        states.intakeDoubleStation = State(0.33_rad, -1.086_rad, 0.01_m, false, -0.7);
//        states.intakeDoubleStation = State(0.23_rad, -1.92_rad, 0.365_m, false, -0.85);

        states.tiltStation = State(1.583_rad, -1.4_rad, 0.015_m);
//        states.idleStation = State(1.65_rad, -1.5_rad, 0.005_m);

        states.scoreConeLow = State(1.80_rad, -0.07_rad, 0.01_m);
        states.scoreConeMid = State(0.63_rad, -1.555_rad, 0.351_m, true, 0.52);
        states.teleopScoreConeMid = State(0.63_rad, -1.555_rad, 0.451_m, true, 0.52);
        states.autoScoreConeMid = State(0.63_rad, -1.52_rad, 0.336_m, true, 0.52);
        states.scoreConeHigh = State(0.68_rad, -71_deg, 1.04_m, true, 0.52);
        states.teleopScoreConeHight = State(0.68_rad, -71_deg, 0.99_m, true, 0.52);

        states.scoreCubeLow = State(1.78_rad, -0.37_rad, 0.01_m);
        states.scoreCubeMid = State(0.255_rad, -1.5_rad, 0.01_m, true, 0.5);
        states.autoScoreCubeMid = State(0.255_rad, -1.45_rad, 0.01_m, true, 0.5);
        states.scoreCubeHigh = State(0.72_rad, -0.86_rad, 0.4_m, true, 0.5);
        states.teleopScoreCubeHigh = State(0.667_rad, -0.86_rad, 0.47_m, true, 0.5);

        states.balancePosition = State(1.65_rad, -1.5_rad, 0.01_m, false, 0.0);

        return states;
    }
};

#endif //BOTBUSTERS_REBIRTH_MANIPULATORSTATE_H
