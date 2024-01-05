// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "SwerveDriveKinematics.h"


namespace botbusters{

    template <size_t NumModules>
    wpi::array<frc::SwerveModuleState, NumModules>
    SwerveDriveKinematics<NumModules>::ToSwerveModuleStates(const frc::ChassisSpeeds &chassisSpeeds,
                                                            const frc::Translation2d &centerOfRotation) const {
            if (chassisSpeeds.vx == 0_mps && chassisSpeeds.vy == 0_mps &&
                chassisSpeeds.omega == 0_rad_per_s) {
//                std::vector<frc::Rotation2d> defaultAngles = {{-45_deg},{45_deg}, {45_deg}, {-45_deg}}; // X mode
//                std::vector<frc::Rotation2d> defaultAngles = {{45_deg},{-45_deg}, {-45_deg}, {45_deg}}; // o mode
                std::vector<frc::Rotation2d> defaultAngles = {{0_deg},{0_deg}, {0_deg}, {0_deg}}; // H mode
                for (size_t i = 0; i < NumModules; i++) {
                    m_moduleStates[i].speed = 0_mps;
//                    m_moduleStates[i].angle = defaultAngles[i];
                }

                return m_moduleStates;
            }

            // We have a new center of rotation. We need to compute the matrix again.
            if (centerOfRotation != m_previousCoR) {
                for (size_t i = 0; i < NumModules; i++) {
                    // clang-format off
                    m_inverseKinematics.template block<2, 3>(i * 2, 0) =
                            frc::Matrixd<2, 3>{
                                    {1, 0, (-m_modules[i].Y() + centerOfRotation.Y()).value()},
                                    {0, 1, (+m_modules[i].X() - centerOfRotation.X()).value()}};
                    // clang-format on
                }
                m_previousCoR = centerOfRotation;
            }

            Eigen::Vector3d chassisSpeedsVector{chassisSpeeds.vx.value(),
                                                chassisSpeeds.vy.value(),
                                                chassisSpeeds.omega.value()};

            frc::Matrixd<NumModules * 2, 1> moduleStateMatrix =
                    m_inverseKinematics * chassisSpeedsVector;

            for (size_t i = 0; i < NumModules; i++) {
                units::meters_per_second_t x{moduleStateMatrix(i * 2, 0)};
                units::meters_per_second_t y{moduleStateMatrix(i * 2 + 1, 0)};

                auto speed = units::math::hypot(x, y);
                frc::Rotation2d rotation{x.value(), y.value()};

                m_moduleStates[i] = {speed, rotation};
            }

            return m_moduleStates;

    }

    template <size_t NumModules>
    template<typename... ModuleStates>
    frc::ChassisSpeeds SwerveDriveKinematics<NumModules>::ToChassisSpeeds(ModuleStates &&...wheelStates) const {
            static_assert(sizeof...(wheelStates) == NumModules,
                          "Number of modules is not consistent with number of wheel "
                          "locations provided in constructor.");

            wpi::array<frc::SwerveModuleState, NumModules> moduleStates{wheelStates...};

            return this->ToChassisSpeeds(moduleStates);
        }

    template<size_t NumModules>
    frc::ChassisSpeeds SwerveDriveKinematics<NumModules>::ToChassisSpeeds(wpi::array<frc::SwerveModuleState, NumModules> moduleStates) const {
            frc::Matrixd<NumModules * 2, 1> moduleStateMatrix;

            for (size_t i = 0; i < NumModules; ++i) {
                frc::SwerveModuleState module = moduleStates[i];
                moduleStateMatrix(i * 2, 0) = module.speed.value() * module.angle.Cos();
                moduleStateMatrix(i * 2 + 1, 0) = module.speed.value() * module.angle.Sin();
            }

            Eigen::Vector3d chassisSpeedsVector =
                    m_forwardKinematics.solve(moduleStateMatrix);

            return {units::meters_per_second_t{chassisSpeedsVector(0)},
                    units::meters_per_second_t{chassisSpeedsVector(1)},
                    units::radians_per_second_t{chassisSpeedsVector(2)}};
        }

    template<size_t NumModules>
    template <typename... ModuleDeltas>
    frc::Twist2d SwerveDriveKinematics<NumModules>::ToTwist2d(ModuleDeltas &&...wheelDeltas) const {
        static_assert(sizeof...(wheelDeltas) == NumModules,
                      "Number of modules is not consistent with number of wheel "
                      "locations provided in constructor.");

        wpi::array<frc::SwerveModulePosition, NumModules> moduleDeltas{wheelDeltas...};
        return this->ToTwist2d(moduleDeltas);
    }

    template <size_t NumModules>
    frc::Twist2d SwerveDriveKinematics<NumModules>::ToTwist2d(
            wpi::array<frc::SwerveModulePosition, NumModules> moduleDeltas) const {
        frc::Matrixd<NumModules * 2, 1> moduleDeltaMatrix;

        for (size_t i = 0; i < NumModules; ++i) {
            frc::SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix(i * 2, 0) = module.distance.value() * module.angle.Cos();
            moduleDeltaMatrix(i * 2 + 1, 0) =
            module.distance.value() * module.angle.Sin();
        }

        Eigen::Vector3d chassisDeltaVector =
                m_forwardKinematics.solve(moduleDeltaMatrix);

        return {units::meter_t{chassisDeltaVector(0)},
                units::meter_t{chassisDeltaVector(1)},
                units::radian_t{chassisDeltaVector(2)}};
    }

    template<size_t NumModules>
    void SwerveDriveKinematics<NumModules>::DesaturateWheelSpeeds(
            wpi::array<frc::SwerveModuleState, NumModules> *moduleStates,
            units::meters_per_second_t attainableMaxSpeed) {
        auto &states = *moduleStates;
        auto realMaxSpeed = std::max_element(states.begin(), states.end(),
                                             [](const auto &a, const auto &b) {
                                                 return units::math::abs(a.speed) <
                                                        units::math::abs(b.speed);
                                             })
                ->speed;

        if (realMaxSpeed > attainableMaxSpeed) {
            for (auto & module : states) {
                module.speed = module.speed / realMaxSpeed * attainableMaxSpeed;
            }
        }
    }

    template<size_t NumModules>
    void SwerveDriveKinematics<NumModules>::DesaturateWheelSpeeds(
            wpi::array<frc::SwerveModuleState, NumModules> *moduleStates, frc::ChassisSpeeds currentChassisSpeed,
            units::meters_per_second_t attainableMaxModuleSpeed,
            units::meters_per_second_t attainableMaxRobotTranslationSpeed,
            units::radians_per_second_t attainableMaxRobotRotationSpeed) {
        auto &states = *moduleStates;

        auto realMaxSpeed = std::max_element(states.begin(), states.end(),
                                             [](const auto &a, const auto &b) {
                                                 return units::math::abs(a.speed) <
                                                        units::math::abs(b.speed);
                                             })
                ->speed;

        if (attainableMaxRobotTranslationSpeed == 0_mps ||
            attainableMaxRobotRotationSpeed == 0_rad_per_s || realMaxSpeed == 0_mps) {
            return;
        }

        auto translationalK =
                units::math::hypot(currentChassisSpeed.vx, currentChassisSpeed.vy) /
                attainableMaxRobotTranslationSpeed;

        auto rotationalK = units::math::abs(currentChassisSpeed.omega) /
                           attainableMaxRobotRotationSpeed;

        auto k = units::math::max(translationalK, rotationalK);

        auto scale = units::math::min(k * attainableMaxModuleSpeed / realMaxSpeed,
                                      units::scalar_t(1));
        for (auto & module : states) {
            module.speed = module.speed *scale;
        }
    }
    template class EXPORT_TEMPLATE_DEFINE(WPILIB_DLLEXPORT)
    SwerveDriveKinematics<4>;
}


