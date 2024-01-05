//
// Created by cc on 10/01/23.
//

#include "SwerveDriveOdometry.h"


namespace botbusters{

    template<size_t NumModules>
    void SwerveDriveOdometry<NumModules>::ResetPosition(const Rotation2d &gyroAngle,
                                                        const wpi::array<SwerveModulePosition, NumModules> &modulePositions,
                                                        const Pose2d &pose) {
        m_pose = pose;
        m_previousAngle = pose.Rotation();
        m_gyroOffset = m_pose.Rotation() - gyroAngle;

        for (size_t i = 0; i < NumModules; i++) {
            m_previousModulePositions[i].distance = modulePositions[i].distance;
        }
    }


    template<size_t NumModules>
    const frc::Pose2d& SwerveDriveOdometry<NumModules>::Update(const Rotation2d &gyroAngle,
                                                        const wpi::array<SwerveModulePosition, NumModules> &modulePositions) {
        auto moduleDeltas =
                wpi::array<SwerveModulePosition, NumModules>(wpi::empty_array);
        for (size_t index = 0; index < NumModules; index++) {
            auto lastPosition = m_previousModulePositions[index];
            auto currentPosition = modulePositions[index];
            moduleDeltas[index] = {currentPosition.distance - lastPosition.distance,
                                   currentPosition.angle};

            m_previousModulePositions[index].distance = modulePositions[index].distance;
        }

        auto angle = gyroAngle + m_gyroOffset;

        auto twist = m_kinematics.ToTwist2d(moduleDeltas);
        twist.dtheta = (angle - m_previousAngle).Radians();

        auto newPose = m_pose.Exp(twist);

        m_previousAngle = angle;
        m_pose = {newPose.Translation(), angle};

        return m_pose;
    }

    template class EXPORT_TEMPLATE_DEFINE(WPILIB_DLLEXPORT) SwerveDriveOdometry<4>;
}