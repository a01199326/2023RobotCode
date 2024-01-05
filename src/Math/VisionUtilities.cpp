//
// Created by abiel on 2/1/22.
//

#include "VisionUtilities.h"
#include <Eigen/SVD>
#include <algorithm>
#include <numeric>
#include <frc/geometry/Rotation2d.h>
#include <iostream>

//https://github.com/DLuensch/Least-Squares-Circle-Fitting-Kasa-Method-/blob/master/src/circleFitting.cpp
bool VisionUtilities::solveLeastSquaresCircle(
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &points,
        Eigen::Vector2d &midpoint, double &radius) {
    size_t length = points.size();
    double x1;
    double x2;
    double x3;
    Eigen::MatrixXd AFill(3, length);
    Eigen::MatrixXd A(length, 3);
    Eigen::VectorXd AFirst(length);
    Eigen::VectorXd ASec(length);
    Eigen::VectorXd AFirstSquared(length);
    Eigen::VectorXd ASecSquared(length);
    Eigen::VectorXd ASquaredRes(length);
    Eigen::VectorXd b(length);
    Eigen::VectorXd c(3);
    bool ok = true;

    if (length > 1) {
        for (size_t i = 0; i < length; i++) {
            AFill(0, i) = points[i](0);
            AFill(1, i) = points[i](1);
            AFill(2, i) = 1;
        }

        A = AFill.transpose();

        for (size_t i = 0; i < length; i++) {
            AFirst(i) = A(i, 0);
            ASec(i) = A(i, 1);
        }

        for (size_t i = 0; i < length; i++) {
            AFirstSquared(i) = AFirst(i) * AFirst(i);
            ASecSquared(i) = ASec(i) * ASec(i);
        }

        b = AFirstSquared + ASecSquared;

        c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        x1 = c(0);
        midpoint(0) = x1 * 0.5;
        x2 = c(1);
        midpoint(1) = x2 * 0.5;
        x3 = c(2);
        radius = sqrt((x1 * x1 + x2 * x2) / 4 + x3);
    } else {
        ok = false;
    }

    return ok;
}

std::optional<VisionUtilities::Circle>
VisionUtilities::solveLeastSquaresCircle(const std::vector<std::pair<double, double>> &points) {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> eigenPoints;
    Eigen::Vector2d midpoint;
    VisionUtilities::Circle circle;

    std::transform(points.begin(), points.end(), std::back_inserter(eigenPoints),
                   [](const std::pair<double, double> &point) {
                       return Eigen::Vector2d(point.first, point.second);
                   });

    bool ok = solveLeastSquaresCircle(eigenPoints, midpoint, circle.radius);
    if (!ok) return std::nullopt;
    circle.midpoint = std::pair(midpoint.x(), midpoint.y());

    return circle;
}

std::pair<double, double>
VisionUtilities::calculateTargetCenter(const std::vector<std::pair<double, double>> &corners) {
    //Get sum of all points
    auto targetCenter = std::accumulate(corners.begin(), corners.end(),
                                        std::pair(0.0, 0.0), [](const auto &p1, const auto &p2) {
                return std::pair(p1.first + p2.first, p1.second + p2.second);
            });
    //Average center
    targetCenter.first /= corners.size();
    targetCenter.second /= corners.size();

    return targetCenter;
}
std::vector<std::pair<double, double>>
VisionUtilities::sortRectanglePoints(const std::vector<std::pair<double, double>> &points) {
    if(points.size() != 4) return {};
    std::vector<std::pair<double,double>> sortedPoints;
    auto center = calculateTargetCenter(points);
    std::vector<std::pair<double, std::pair<double,double>>> positivePolarCoords, negativePolarCoords; //(angle, (x,y))
    //Gets polar coordinates of all points, using the center as the origin
    std::for_each(points.begin(), points.end(),
                   [&](const std::pair<double, double> &point) {
                        auto rot = frc::Rotation2d(point.first - center.first, point.second - center.second) - frc::Rotation2d(units::degree_t(90));
                        auto angle = rot.Radians().value();
                        if(angle > 0) positivePolarCoords.emplace_back(angle, point);
                        else negativePolarCoords.emplace_back(angle, point);
                   });

    //Top left should be max positive value, top right should be min positive value
    auto [bottomRight, topLeft] = std::minmax_element(positivePolarCoords.begin(), positivePolarCoords.end(), [](const auto &a, const auto &b){
       return a.first < b.first;
    });

    //Bottom left should be min negative value, bottom right should be max negative value
    auto [topRight, bottomLeft] = std::minmax_element(negativePolarCoords.begin(), negativePolarCoords.end(), [](const auto &a, const auto &b){
        return a.first < b.first;
    });

    return {topLeft->second, topRight->second, bottomLeft->second, bottomRight->second};
}

std::optional<frc::Translation2d> VisionUtilities::cameraToTargetTranslation(std::pair<double, double> corner,
                                                              const std::pair<double, double> &cameraResolution,
                                                              double vpw,
                                                              double vph,
                                                              double cameraHeight,
                                                              double cameraPitch,
                                                              double goalHeight) {
    double yPixels = corner.first;
    double zPixels = corner.second;

    // SetRobotPose frame of reference
    double nY = -((yPixels - (cameraResolution.first / 2)) / (cameraResolution.first / 2));
    double nZ = -((zPixels - (cameraResolution.second / 2)) / (cameraResolution.second / 2));

    frc::Translation2d xzPlaneTranslation =
        frc::Translation2d(1.0_m, units::meter_t(vph / 2.0 * nZ)).RotateBy(units::degree_t(cameraPitch));
    double x = xzPlaneTranslation.X().value();
    double y = vpw / 2.0 * nY;
    double z = xzPlaneTranslation.Y().value();

    double differentialHeight = cameraHeight - goalHeight;
    if ((z < 0.0) == (differentialHeight > 0.0)) {
      double scaling = differentialHeight / -z;
      double distance = std::hypot(x, y) * scaling;
      frc::Rotation2d angle = frc::Rotation2d(x, y);
      return frc::Translation2d(units::meter_t(distance * angle.Cos()),
          units::meter_t(distance * angle.Sin()));
    }
    return {};    
}

void VisionUtilities::publishCircle(nt::NetworkTableEntry entry, nt::NetworkTableEntry midpointEntry, const Circle &circle) {
    entry.SetString(fmt::format("C({},{})R{}C", circle.midpoint.first, circle.midpoint.second, circle.radius));
    VisionUtilities::publishPoint(midpointEntry, circle.midpoint);
}

void VisionUtilities::publishPoints(nt::NetworkTableEntry entry, const std::vector<std::pair<double,double>> &points){
    std::vector<std::string> strings;
    std::transform(points.begin(), points.end(), std::back_inserter(strings), [](const auto &point){
       return fmt::format("({}[{})", point.first, point.second);
    });

    entry.SetStringArray(strings);
}

void VisionUtilities::publishPoint(nt::NetworkTableEntry entry, const std::pair<double, double> &points) {
    //entry.SetDoubleArray({points.first, points.second});
    entry.SetString(fmt::format("({}[{})", points.first, points.second));
}

frc::Translation3d VisionUtilities::calculateR_FSFromC_TS(const frc::Transform3d &cameraPoseToTarget,
                                                          frc::Pose3d &cameraPoseToRobot,
                                                          const frc::Pose3d &tagPoseToField) {

    return {};
}

void VisionUtilities::publishVector(nt::NetworkTableEntry entry, const std::vector<int> &vector) {
    std::string message;
    for (auto &item : vector){
        message.append(fmt::format("{}, ", item));
    }
    entry.SetString(message);
}

units::meter_t VisionUtilities::calculateXOffset(const TrackedTarget &target, const VisionManagerConfig &config,
                                                 int cameraId) {
    auto cameraPose = config.cameraPoses[cameraId];
    units::radian_t floorAngle = cameraPose.Rotation().Y() + target.tY;
    auto out = units::meter_t((config.midPoleHigh.value() - 0.196) / std::tan(floorAngle.value()));
    return out;

}

units::meter_t VisionUtilities::calculateYOffset(const TrackedTarget &target, const std::shared_ptr<EctoSwerve> &swerve, const VisionManagerConfig &config, int cameraId){
    auto cameraPose = config.cameraPoses[cameraId];
    units::radian_t floorAngle = units::radian_t(swerve->getYaw()) - target.tX;
    auto out = units::meter_t(calculateXOffset(target, config, cameraId).value() * std::tan(floorAngle.value()));
    return out;
}

frc::Transform3d VisionUtilities::openCVToWPILib(std::vector<double> tVec, std::vector<double> rVec) {
    if (tVec.empty() || rVec.empty()){
        return {};
    }
    return {
            frc::Translation3d(tVec[2] * 1_m, -tVec[0] * 1_m, -tVec[1] * 1_m),
            frc::Rotation3d({rVec[2], -rVec[0], -rVec[1]},
                            units::radian_t(std::sqrt(
                                    std::pow(rVec[0], 2) +
                                    std::pow(rVec[1], 2) +
                                    std::pow(rVec[2], 2)
                            ))

            )
    };
}

frc::Transform3d VisionUtilities::vectorToTransform3d(std::vector<double> v) {
    if (v.empty()) return {};
    return {
            {
                units::meter_t(v[0]),
                units::meter_t(v[1]),
                units::meter_t(v[2])
            },
            {
                units::degree_t(v[3]),
                units::degree_t(v[4]),
                units::degree_t(v[5])
            }
    };
}

bool VisionUtilities::filterTagPose(const AprilTag &tag, const VisionManagerConfig &config, const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<spdlog::logger> &log) {
    auto tagPose = config.tagPoses.tag16h5.find(tag.tagId)->second;
    auto tagPoseInRobotInFieldSpace = tag.r_fs.Translation().ToTranslation2d();

    bool distanceToTag = tagPose.Translation().Distance(swerve->getPose().Translation()) < 4.5_m;
    auto distToPose = tag.r_fs.Translation().ToTranslation2d().Distance(swerve->getPose().Translation()) < 16_m;
    bool isInRegion = EctoMath::isPoseInRegion({tag.r_fs.Translation().ToTranslation2d(), 0_deg}, {0_m, 0_m}, {16.459_m, 8.23_m});
//    bool isInRegion = false;
    bool angularSpeed = swerve->getChassisSpeeds().omega < 70_deg_per_s;
    double angularDiff = EctoMath::wrapAngle(std::abs((tag.r_fs.Rotation().ToRotation2d().Radians() - tag.robotYaw).value()));
    bool angularRange = angularDiff < 0.05;
    bool height = std::abs(tag.r_fs.Z().value()) < 0.4;
//    std::cout << fmt::format("tagAngle: {}, robotAngle: {}, diff: {}", tag.r_fs.ToPose2d().Rotation().Radians().value(), tag.robotYaw.value(), EctoMath::wrapAngle(angularDiff)) << std::endl;
//    std::cout << fmt::format("tagMeasuredY: {}, prcntOfImage: {}", tag.r_fs.ToPose2d().Y().value(), tag.areaOfImage) << std::endl;
//    bool filter = distanceToTag && distToPose && min && max && angularSpeed && angularRange;
    bool filter = distToPose && isInRegion && angularSpeed && angularRange;
//    frc::SmartDashboard::PutString(fmt::format("{}/ERROR", tag.id), fmt::format("distToPose: {}, isInRegion: {}, angularSpeeds: {}, angularRange: {}", distToPose, isInRegion, angularSpeed, angularRange));
//    std::cout << fmt::format("distToPose: {}, isInRegion: {}, angularSpeeds: {}, angularRange: {}", distToPose, isInRegion, angularSpeed, angularRange) << std::endl;

    return filter;
}

bool VisionUtilities::filterPoseOnAchievableVariance(frc::Translation2d &translation,
                                                     const std::shared_ptr<EctoSwerve> &swerve, units::second_t dt) {

    double velMag = swerve->getVelMagitude();
    double possibleMoveMent = velMag * dt.value();
    double dist = translation.Distance(swerve->getPose().Translation()).value();
    return dist < possibleMoveMent;

}

void VisionUtilities::publishTag(const AprilTag &tag, const std::shared_ptr<nt::NetworkTable> table) {
//    table->GetEntry(fmt::format("{}/EstimatedR_FS/x", tag.id)).SetDouble(tag.r_fs.X().value());
//    table->GetEntry(fmt::format("{}/EstimatedR_FS/y", tag.id)).SetDouble(tag.r_fs.Y().value());
//    table->GetEntry(fmt::format("{}/EstimatedR_FS/z", tag.id)).SetDouble(tag.r_fs.Z().value());
//    table->GetEntry(fmt::format("{}/EstimatedR_FS/point", tag.id)).SetString(fmt::format("({}, {}, {})", tag.r_fs.X().value(),tag.r_fs.Y().value(),tag.r_fs.Z().value()));
//    table->GetEntry(fmt::format("{}/EstimatedR_FS/yaw", tag.id)).SetDouble(tag.r_fs.ToPose2d().Rotation().Degrees().value());
//
//    table->GetEntry(fmt::format("{}/Id", tag.id)).SetDouble(tag.id);
//    table->GetEntry(fmt::format("{}/latency", tag.id)).SetDouble(tag.latency.value());
//    table->GetEntry(fmt::format("{}/timestamp", tag.id)).SetDouble(tag.timestamp.value());
//    table->GetEntry(fmt::format("{}/robotYaw", tag.id)).SetDouble(tag.robotYaw.value());
//
//    auto tagPose = std::make_shared<frc::Field2d>();
//    frc::SmartDashboard::PutData(fmt::format("{} Pose", tag.id), tagPose.get());
//    tagPose->SetRobotPose(tag.r_fs.ToPose2d());

}

double VisionUtilities::calculateTagWeight(const AprilTag &tag, const VisionManagerConfig &config,
                                           const std::shared_ptr<EctoSwerve> &swerve,
                                           const std::shared_ptr<spdlog::logger> &log) {
    auto tagPoseInField = config.tagPoses.tag16h5.find(tag.tagId)->second;
    auto tagPoseR_FS = tag.r_fs;
    auto speeds = swerve->getChassisSpeeds();
//
//    double distanceToTag EctoMath::map()
//    double distToPose
//    double isInRegion
//    double angularSpeed = EctoMath::map(speeds.omega, 0.0, )
//    double angularDiff
//    double angularRange

    return 0;
}