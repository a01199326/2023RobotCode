// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <utility>

#include "frc/kinematics/ChassisSpeeds.h"
#include "units/math.h"
#include <frc/geometry/Translation2d.h>
#include <cstddef>

#include <wpi/SymbolExports.h>
#include <wpi/array.h>

#include "Eigen/QR"
#include "frc/EigenCore.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/velocity.h"
#include "wpimath/MathShared.h"

namespace botbusters{
template <size_t NumModules>
class SwerveDriveKinematics {
public:

    template<typename... Wheels>
    explicit SwerveDriveKinematics(frc::Translation2d wheel, Wheels &&... wheels)
            : m_modules{wheel, wheels...}, m_moduleStates(wpi::empty_array) {
        static_assert(sizeof...(wheels) >= 1,
                      "A swerve drive requires at least two modules");

        for (size_t i = 0; i < NumModules; i++) {
            // clang-format off
            m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
                    1, 0, (-m_modules[i].Y()).value(),
                    0, 1, (+m_modules[i].X()).value();
            // clang-format on
        }

        m_forwardKinematics = m_inverseKinematics.householderQr();

        wpi::math::MathSharedStore::ReportUsage(
                wpi::math::MathUsageId::kKinematics_SwerveDrive, 1);
    }

    explicit SwerveDriveKinematics(
            const wpi::array<frc::Translation2d, NumModules> &wheels)
            : m_modules{wheels}, m_moduleStates(wpi::empty_array) {
        for (size_t i = 0; i < NumModules; i++) {
            // clang-format off
            m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
                    1, 0, (-m_modules[i].Y()).value(),
                    0, 1, (+m_modules[i].X()).value();
            // clang-format on
        }

        m_forwardKinematics = m_inverseKinematics.householderQr();

        wpi::math::MathSharedStore::ReportUsage(
                wpi::math::MathUsageId::kKinematics_SwerveDrive, 1);
    }

    SwerveDriveKinematics(const SwerveDriveKinematics &) = default;

    wpi::array<frc::SwerveModuleState, NumModules>
    ToSwerveModuleStates(
            const frc::ChassisSpeeds &chassisSpeeds,
            const frc::Translation2d &centerOfRotation = frc::Translation2d(0_m, 0_m)) const;

    template<typename... ModuleStates>
    frc::ChassisSpeeds ToChassisSpeeds(
            ModuleStates &&... wheelStates) const;

    frc::ChassisSpeeds ToChassisSpeeds(
            wpi::array<frc::SwerveModuleState, NumModules> moduleStates) const;

    template <typename... ModuleDeltas>
    frc::Twist2d ToTwist2d(ModuleDeltas&&... wheelDeltas) const;


    frc::Twist2d ToTwist2d(
            wpi::array<frc::SwerveModulePosition, NumModules> wheelDeltas) const;


    void DesaturateWheelSpeeds(
            wpi::array<frc::SwerveModuleState, NumModules> *moduleStates,
            units::meters_per_second_t attainableMaxSpeed);

    void DesaturateWheelSpeeds(
            wpi::array<frc::SwerveModuleState, NumModules> *moduleStates,
            frc::ChassisSpeeds currentChassisSpeed,
            units::meters_per_second_t attainableMaxModuleSpeed,
            units::meters_per_second_t attainableMaxRobotTranslationSpeed,
            units::radians_per_second_t attainableMaxRobotRotationSpeed);

private:
    mutable frc::Matrixd<NumModules * 2, 3> m_inverseKinematics;
    Eigen::HouseholderQR<frc::Matrixd<NumModules * 2, 3>> m_forwardKinematics;
    wpi::array<frc::Translation2d, NumModules> m_modules;
    mutable wpi::array<frc::SwerveModuleState, NumModules> m_moduleStates;

    mutable frc::Translation2d m_previousCoR;
};
    extern template class EXPORT_TEMPLATE_DECLARE(WPILIB_DLLEXPORT)
    SwerveDriveKinematics<4>;

}