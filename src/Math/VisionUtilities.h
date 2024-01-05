//
// Created by abiel on 2/1/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONUTILITIES_H
#define BOTBUSTERS_REBIRTH_VISIONUTILITIES_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <optional>
#include <frc/geometry/Translation2d.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/VisionManager/Sources/VisionSource.h"
#include <frc/geometry/Pose3d.h>
#include "Core/VisionManager/VisionManager.h"
//#include "Core/EctoModule/Module.h"


class VisionUtilities {
public:
    struct Circle {
        std::pair<double,double> midpoint;
        double radius;
    };

    static std::optional<Circle> solveLeastSquaresCircle(const std::vector<std::pair<double,double>> &points);

    static std::pair<double,double> calculateTargetCenter(const std::vector<std::pair<double,double>> &corners);
    //Returns: topLeft, topRight, bottomLeft, bottomRight
    static std::vector<std::pair<double,double>> sortRectanglePoints(const std::vector<std::pair<double,double>> &points);

    /**
     *
     * @param corner
     * @param cameraResolution
     * @param vpw 59.6 for limelight
     * @param vph 45.7 for limelight
     * @param cameraPitch
     * @param goalHeight
     * @return
     */
    static std::optional<frc::Translation2d> cameraToTargetTranslation(std::pair<double, double> corner,
                                                                       const std::pair<double, double> &cameraResolution,
                                                                       double vpw,
                                                                       double vph,
                                                                       double cameraHeight,
                                                                       double cameraPitch,
                                                                       double goalHeight);

    static void publishCircle(nt::NetworkTableEntry entry, nt::NetworkTableEntry midpointEntry, const Circle &circle);

    static void publishPoints(nt::NetworkTableEntry entry, const std::vector<std::pair<double,double>> &points);

    static void publishPoint(nt::NetworkTableEntry entry, const std::pair<double,double> &points);

    static frc::Translation3d calculateR_FSFromC_TS(const frc::Transform3d &cameraPoseToTarget, frc::Pose3d &cameraPoseToRobot, const frc::Pose3d &tagPoseToField);

    static bool filterTagPose(const AprilTag &tag, const VisionManagerConfig &config, const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<spdlog::logger> &log);

    static void publishVector(nt::NetworkTableEntry entry, const std::vector<int> &vector);

    static frc::Transform3d openCVToWPILib(std::vector<double> tVec, std::vector<double> rVec);

    static frc::Transform3d openCVToWPILib(std::vector<double> rtVec){
        if (rtVec.empty()){
            return{};
        }
        return openCVToWPILib({rtVec[0], rtVec[1], rtVec[2]}, {rtVec[3], rtVec[4], rtVec[5]});
    }

    static frc::Transform3d vectorToTransform3d(std::vector<double> v);

    static bool filterPoseOnAchievableVariance(frc::Translation2d &translation, const std::shared_ptr<EctoSwerve> &swerve, units::second_t dt);

    static units::meter_t calculateYOffset(const TrackedTarget &target, const std::shared_ptr<EctoSwerve> &swerve, const VisionManagerConfig &config, int cameraId);

    static units::meter_t calculateXOffset(const TrackedTarget &target, const VisionManagerConfig &config, int cameraId);

    static void publishTag(const AprilTag &tag, const std::shared_ptr<nt::NetworkTable> table);

    static double calculateTagWeight(const AprilTag &tag, const VisionManagerConfig &config, const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<spdlog::logger> &log);

    template <typename Vector1, typename Vector2>
    static auto Dot(const Vector1& a, const Vector2& b) -> decltype(auto) {
        // (a_x i + a_y j) . (b_x i + b_y j)
        // = a_x b_x + a_y b_y
        return a.X() * b.X() + a.Y() * b.Y();
    }

private:
    static bool solveLeastSquaresCircle(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &points, Eigen::Vector2d &midpoint, double &radius);


};

#endif //BOTBUSTERS_REBIRTH_VISIONUTILITIES_H
