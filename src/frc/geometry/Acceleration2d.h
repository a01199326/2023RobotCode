// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/geometry/Rotation2d.h"
#include "units/acceleration.h"

namespace wpi {
    class json;
}  // namespace wpi

namespace frc {

/**
 * Represents a Acceleration in 2d space.
 * This object can be used to represent a point or a vector.
 *
 * This assumes that you are using conventional mathematical axes.
 * When the robot is placed on the origin, facing toward the X direction,
 * moving forward increases the X, whereas moving to the left increases the Y.
 */
    class Acceleration2d {
    public:
        /**
         * Constructs a Acceleration2d with X and Y components equal to zero.
         */
        constexpr Acceleration2d() = default;

        /**
         * Constructs a Acceleration2d with the X and Y components equal to the
         * provided values.
         *
         * @param x The x component of the acceleration.
         * @param y The y component of the acceleration.
         */
        Acceleration2d(units::meters_per_second_squared_t x, units::meters_per_second_squared_t y);

        /**
         * Constructs a Acceleration2d with the provided acceleration and angle. This is
         * essentially converting from polar coordinates to Cartesian coordinates.
         *
         * @param acceleration The acceleration from the origin to the end of the acceleration.
         * @param angle The angle between the x-axis and the acceleration vector.
         */
        Acceleration2d(units::meters_per_second_squared_t acceleration, const Rotation2d& angle);

        /**
         * Calculates the acceleration between two velocities in 2d space.
         *
         * This function uses the pythagorean theorem to calculate the acceleration.
         * acceleration = std::sqrt((x2 - x1)^2 + (y2 - y1)^2)
         *
         * @param other The acceleration to compute the acceleration to.
         *
         * @return The acceleration between the two velocities.
         */
        units::meters_per_second_squared_t Distance(const Acceleration2d& other) const;

        /**
         * Returns the X component of the acceleration.
         *
         * @return The x component of the acceleration.
         */
        units::meters_per_second_squared_t X() const { return m_x; }

        /**
         * Returns the Y component of the acceleration.
         *
         * @return The y component of the acceleration.
         */
        units::meters_per_second_squared_t Y() const { return m_y; }

        /**
         * Returns the norm, or acceleration from the origin to the acceleration.
         *
         * @return The norm of the acceleration.
         */
        units::meters_per_second_squared_t Norm() const;

        /**
         * Applies a rotation to the acceleration in 2d space.
         *
         * This multiplies the acceleration vector by a counterclockwise rotation
         * matrix of the given angle.
         *
         * [x_new]   [other.cos, -other.sin][x]
         * [y_new] = [other.sin,  other.cos][y]
         *
         * For example, rotating a Acceleration2d of {2, 0} by 90 degrees will return a
         * Acceleration2d of {0, 2}.
         *
         * @param other The rotation to rotate the acceleration by.
         *
         * @return The new rotated acceleration.
         */
        Acceleration2d RotateBy(const Rotation2d& other) const;

        /**
         * Adds two velocities in 2d space and returns the sum. This is similar to
         * vector addition.
         *
         * For example, Acceleration2d{1.0, 2.5} + Acceleration2d{2.0, 5.5} =
         * Acceleration2d{3.0, 8.0}
         *
         * @param other The acceleration to add.
         *
         * @return The sum of the velocities.
         */
        Acceleration2d operator+(const Acceleration2d& other) const;

        /**
         * Adds the new acceleration to the current acceleration.
         *
         * This is similar to the + operator, except that the current object is
         * mutated.
         *
         * @param other The acceleration to add.
         *
         * @return The reference to the new mutated object.
         */
        Acceleration2d& operator+=(const Acceleration2d& other);

        /**
         * Subtracts the other acceleration from the other acceleration and returns the
         * difference.
         *
         * For example, Acceleration2d{5.0, 4.0} - Acceleration2d{1.0, 2.0} =
         * Acceleration2d{4.0, 2.0}
         *
         * @param other The acceleration to subtract.
         *
         * @return The difference between the two velocities.
         */
        Acceleration2d operator-(const Acceleration2d& other) const;

        /**
         * Subtracts the new acceleration from the current acceleration.
         *
         * This is similar to the - operator, except that the current object is
         * mutated.
         *
         * @param other The acceleration to subtract.
         *
         * @return The reference to the new mutated object.
         */
        Acceleration2d& operator-=(const Acceleration2d& other);

        /**
         * Returns the inverse of the current acceleration. This is equivalent to
         * rotating by 180 degrees, flipping the point over both axes, or simply
         * negating both components of the acceleration.
         *
         * @return The inverse of the current acceleration.
         */
        Acceleration2d operator-() const;

        /**
         * Multiplies the acceleration by a scalar and returns the new acceleration.
         *
         * For example, Acceleration2d{2.0, 2.5} * 2 = Acceleration2d{4.0, 5.0}
         *
         * @param scalar The scalar to multiply by.
         *
         * @return The scaled acceleration.
         */
        Acceleration2d operator*(double scalar) const;

        /**
         * Multiplies the current acceleration by a scalar.
         *
         * This is similar to the * operator, except that current object is mutated.
         *
         * @param scalar The scalar to multiply by.
         *
         * @return The reference to the new mutated object.
         */
        Acceleration2d& operator*=(double scalar);

        /**
         * Divides the acceleration by a scalar and returns the new acceleration.
         *
         * For example, Acceleration2d{2.0, 2.5} / 2 = Acceleration2d{1.0, 1.25}
         *
         * @param scalar The scalar to divide by.
         *
         * @return The scaled acceleration.
         */
        Acceleration2d operator/(double scalar) const;

        /**
         * Checks equality between this Acceleration2d and another object.
         *
         * @param other The other object.
         * @return Whether the two objects are equal.
         */
        bool operator==(const Acceleration2d& other) const;

        /**
         * Checks inequality between this Acceleration2d and another object.
         *
         * @param other The other object.
         * @return Whether the two objects are not equal.
         */
        bool operator!=(const Acceleration2d& other) const;

        /**
         * Divides the current acceleration by a scalar.
         *
         * This is similar to the / operator, except that current object is mutated.
         *
         * @param scalar The scalar to divide by.
         *
         * @return The reference to the new mutated object.
         */
        Acceleration2d& operator/=(double scalar);

    private:
        units::meters_per_second_squared_t m_x = 0_mps_sq;
        units::meters_per_second_squared_t m_y = 0_mps_sq;
    };

    void to_json(wpi::json& json, const Acceleration2d& state);

    void from_json(const wpi::json& json, Acceleration2d& state);

}  // namespace frc
