//
// Created by abiel on 2/12/20.
//

#ifndef BOTBUSTERSREBIRTH_VISIONMANAGER_H
#define BOTBUSTERSREBIRTH_VISIONMANAGER_H

#include "Core/EctoModule/WPISubsystem.h"
#include <Systems/EctoSwerve/EctoSwerve.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <optional>
#include <functional>
#include <frc/Notifier.h>
#include "frc/geometry/Velocity2d.h"
#include "Core/VisionManager/Sources/VisionSource.h"
#include "TagPoses.h"
#include "Math/InterpolatingTable/InterpolatingTable.h"
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/geometry/Pose3d.h>

struct VisionManagerConfig{

    frc::Pose2d robotPose{{0_m, 0_m}, 0_rad};

    frc::Rotation2d robotYaw{0_rad};

    //TODO actually use these lol
    wpi::array<double, 3> statesStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 1> gyroEncoderStdDevs{0.0};
    wpi::array<double, 3> autoVisionStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 3> teleopVisionStdDevs{0.0, 0.0, 0.0};


    frc::Pose2d fieldToTarget{}; //Target position in field coord system
    units::meter_t targetHeight{0_m}; //Target height from floor

    std::vector<frc::Pose3d> cameraPoses;
    //Pitch is from the horizontal plane.
    //The rest is from the robot odometry zero


    aprilTags::TagPoses tagPoses;

    units::meter_t midPoleHigh = 1_mm;
    units::meter_t highPoleHeight = 1_mm;
    units::meter_t tagHeights = 1_mm;

    std::vector<std::pair<double, double>> visionStdDevs;

};

class VisionManager : public WPISubsystem {
public:
    VisionManager(const std::shared_ptr<EctoSwerve> &swerve,
                  const std::vector<std::shared_ptr<VisionSource>> &sources,
                  VisionManagerConfig &config);

    void autoInit() override;

    void teleopInit() override;

    void robotUpdate() override;

    //Returns estimated error to target (very experimental)

    void setCameraToRobot(frc::Transform2d cameraToRobotPose){
        this->cameraToRobot = cameraToRobotPose;
    }

    frc::Pose2d getVisionPose() const;

    frc::Twist2d getVelocityToTarget() const;

    //Use estimated measurements or vision measurements
    double getTargetDistance() const;

    bool hasValidTarget() const {
        return frc::Timer::GetFPGATimestamp() - targetTimestamp < 100_ms; //Invalid target if has not updated within 100ms
    }

    void ignoreVision(bool ignoreVision);

    void setPipeline(int cameraId, int pipelineId);

    void setLeds(bool state, int cameraIndex);


    std::vector<std::shared_ptr<ChecklistItem>> createTests() override;


    std::optional<units::meter_t> getYOffsetToTarget(int cameraId) const;

    std::optional<units::meter_t> getXOffsetToTarget(int cameraId) const;

    void setVisionMinMaxStdDevs(double min, double max) {
        minVisionStdDev = min;
        maxVisionStdDev = max;
    }

private:
    VisionManagerConfig config;

    std::vector<std::shared_ptr<VisionSource>> sources{};
    std::shared_ptr<EctoSwerve> swerve;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("VisionManager");
    std::shared_ptr<nt::NetworkTable> visionTable = ntInstance.GetTable("VisionManager/Vision");
    std::shared_ptr<nt::NetworkTable> tagTable = ntInstance.GetTable("aprilTags");


    units::second_t lastVisionRunTime;

    frc::Pose2d visionPose;

    units::second_t targetTimestamp{0};


    void updateVisionTelemetry(const std::vector<AprilTagResults> &results, const CameraInfo &cameraInfo);

    void updateTagVision(const std::vector<AprilTagResults> &results);

    std::unique_ptr<frc::Notifier> visionNotifier;

    AprilTagResults prevTagRes;

    frc::Field2d visionPoseField;

    /**
     * Target velocity tracking
     */
    frc::Twist2d twistDelta;
    frc::Twist2d lastTwistToTarget;
    units::second_t lastTranslationToTargetTime;

    size_t processedFrames{0}, droppedFrames{0};

    template <typename Vector1, typename Vector2>
    static auto Cross(const Vector1& a, const Vector2& b) -> decltype(auto) {
        // (a_x i + a_y j) x (b_x i + b_y j)
        // = a_x b_y - a_y b_x
        return a.X() * b.Y() - a.Y() * b.X();
    }

    frc::Transform2d cameraToRobot;

    bool useVision{true};

//    frc::LinearFilter<double> xFilter = frc::LinearFilter<double>::SinglePoleIIR(0.45, 5_ms);
//    frc::LinearFilter<double> yFilter = frc::LinearFilter<double>::SinglePoleIIR(0.45, 5_ms);

    aprilTags::TagPoses tagPoses;

    frc::Pose2d tagPose;

    std::shared_ptr<nt::NetworkTable> tagsTable = ntInstance.GetTable("tagData");

    double minVisionStdDev = 0.0;
    double maxVisionStdDev = 0.0;

    std::shared_ptr<InterpolatingTable> visionStdDevs;


};

#endif//BOTBUSTERSREBIRTH_VISIONMANAGER_H
