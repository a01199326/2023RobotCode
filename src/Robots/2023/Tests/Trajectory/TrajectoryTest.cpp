//
// Created by cc on 6/12/23.
//

#include "TrajectoryTest.h"
#include "Control/Path/Trajectory/WPITrajectory.h"

TrajectoryTest::TrajectoryTest() : EctoRobot("TrajectoryTest") {
    ;
}

void TrajectoryTest::robotInit() {
    table->GetEntry("setLeds").SetBoolean(false);
    traj = std::make_shared<ChoreoTrajectory>(botbusters::Trajectory::readJSON("FirstTest.traj"));

    fieldTest =  std::make_shared<frc::Field2d>();
    autoPose = std::make_shared<frc::Field2d>();

    frc::TrajectoryConfig cubeConf(units::meters_per_second_t(4.7),
                                   units::meters_per_second_squared_t(3.25));
//    traj = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
//            {
//                    frc::Pose2d(14.662_m, 4.97_m, {-1.0, 0}),
//                    frc::Pose2d(12.445_m, 4.75_m, {-1.0, 0.0}),
//                    frc::Pose2d(9.42_m, 4.4_m, {-1.0, 0.0})
//            },
//            cubeConf));

    fieldTest->GetObject("Trajectory")->SetTrajectory(wpiTraj);
    autoPose->SetRobotPose(frc::Pose2d());
    frc::SmartDashboard::PutData("thisPose", autoPose.get());
    frc::SmartDashboard::PutData("PublishedTraj", fieldTest.get());
}

void TrajectoryTest::robotUpdate() {
    ;
}

void TrajectoryTest::autoInit() {
    startTime = frc::Timer::GetFPGATimestamp();
}

void TrajectoryTest::autoUpdate() {
    auto time = frc::Timer::GetFPGATimestamp() - startTime;
    auto sample = traj->sample(time);
    lastTime = frc::Timer::GetFPGATimestamp();
    autoPose->SetRobotPose(sample.pose);



}