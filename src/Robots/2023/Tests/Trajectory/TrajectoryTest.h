//
// Created by cc on 6/12/23.
//

#ifndef BOTBUSTERS_REBIRTH_TRAJECTORYTEST_H
#define BOTBUSTERS_REBIRTH_TRAJECTORYTEST_H

#include "Core/EctoRobot.h"

#include <frc/smartdashboard/Field2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include  <frc/trajectory/TrajectoryConfig.h>
#include "Control/TrajectoryGenerator.h"
#include "Control/Path/Trajectory/Trajectory.h"
#include "Control/Path/Trajectory/ChoreoTrajectory.h"



class TrajectoryTest : public EctoRobot{
public:
    TrajectoryTest();

    void robotInit() override;
    void robotUpdate() override;
    void autoUpdate() override;
    void autoInit() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {{EctoMotorType::SparkMax, "front_left_wheel",     7},
                {EctoMotorType::SparkMax, "front_right_wheel",    5},
                {EctoMotorType::SparkMax, "back_left_wheel",      1},
                {EctoMotorType::SparkMax, "back_right_wheel",     3},

                {EctoMotorType::SparkMax, "front_left_steer",     8},
                {EctoMotorType::SparkMax, "front_right_steer",    6},
                {EctoMotorType::SparkMax, "back_left_steer",      2},
                {EctoMotorType::SparkMax, "back_right_steer",     4},

                {EctoMotorType::SparkMax, "telescopic", 10},
                {EctoMotorType::SparkMax, "telescopicFollower", 11},
                {EctoMotorType::SparkMax, "telescopicSecondFollower", 12},

                {EctoMotorType::SparkMax, "arm",                    13},
                {EctoMotorType::SparkMax, "armFollower",            14},


                {EctoMotorType::SparkMax, "manipulator",                    15},

                {EctoMotorType::SparkMax, "wrist",               16},
        };
    }
    std::list<PistonInfo> getPistonConfig(){
        return {
                {"manipulator", 21, frc::PneumaticsModuleType::REVPH, 6, 5}// originali 1, 0 the 14 15, 4, 5
        };
    }
private:
    enum class RobotSide{
        kFront,
        kBack
    };

    //managers
    InputManager &input = InputManager::getInstance();
    PCMManager &pcm = PCMManager::getInstance();

    std::shared_ptr<frc::Field2d> fieldTest;
    std::shared_ptr<frc::Field2d> autoPose;

    units::second_t startTime;

    std::shared_ptr<botbusters::Trajectory> traj;
    frc::Trajectory wpiTraj;

    units::second_t lastTime = 0_s;

    //nt
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table =
            ntInstance.GetTable("TrajectoryTest");
};

#endif //BOTBUSTERS_REBIRTH_TRAJECTORYTEST_H
