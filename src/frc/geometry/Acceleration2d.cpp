// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Acceleration2d.h"

#include <wpi/json.h>

#include "units/math.h"

using namespace frc;

Acceleration2d::Acceleration2d(units::meters_per_second_squared_t x,
                       units::meters_per_second_squared_t y)
        : m_x(x), m_y(y) {}

Acceleration2d::Acceleration2d(units::meters_per_second_squared_t acceleration,
                       const Rotation2d& angle)
        : m_x(acceleration * angle.Cos()), m_y(acceleration * angle.Sin()) {}

units::meters_per_second_squared_t Acceleration2d::Distance(const Acceleration2d& other) const {
    return units::math::hypot(other.m_x - m_x, other.m_y - m_y);
}

units::meters_per_second_squared_t Acceleration2d::Norm() const {
    return units::math::hypot(m_x, m_y);
}

Acceleration2d Acceleration2d::RotateBy(const Rotation2d& other) const {
    return {m_x * other.Cos() - m_y * other.Sin(),
            m_x * other.Sin() + m_y * other.Cos()};
}

Acceleration2d Acceleration2d::operator+(const Acceleration2d& other) const {
    return {X() + other.X(), Y() + other.Y()};
}

Acceleration2d& Acceleration2d::operator+=(const Acceleration2d& other) {
    m_x += other.m_x;
    m_y += other.m_y;
    return *this;
}

Acceleration2d Acceleration2d::operator-(const Acceleration2d& other) const {
    return *this + -other;
}

Acceleration2d& Acceleration2d::operator-=(const Acceleration2d& other) {
    *this += -other;
    return *this;
}

Acceleration2d Acceleration2d::operator-() const { return {-m_x, -m_y}; }

Acceleration2d Acceleration2d::operator*(double scalar) const {
    return {scalar * m_x, scalar * m_y};
}

Acceleration2d& Acceleration2d::operator*=(double scalar) {
    m_x *= scalar;
    m_y *= scalar;
    return *this;
}

Acceleration2d Acceleration2d::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

bool Acceleration2d::operator==(const Acceleration2d& other) const {
    return units::math::abs(m_x - other.m_x) < 1E-9_mps_sq &&
           units::math::abs(m_y - other.m_y) < 1E-9_mps_sq;
}

bool Acceleration2d::operator!=(const Acceleration2d& other) const {
    return !operator==(other);
}

Acceleration2d& Acceleration2d::operator/=(double scalar) {
    *this *= (1.0 / scalar);
    return *this;
}

void frc::to_json(wpi::json& json, const Acceleration2d& acceleration) {
    json = wpi::json{{"x", acceleration.X().value()}, {"y", acceleration.Y().value()}};
}

void frc::from_json(const wpi::json& json, Acceleration2d& acceleration) {
    acceleration = Acceleration2d{units::meters_per_second_squared_t{json.at("x").get<double>()},
                          units::meters_per_second_squared_t{json.at("y").get<double>()}};
}
