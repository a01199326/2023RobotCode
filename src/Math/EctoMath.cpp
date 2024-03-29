//
// Created by Abiel on 10/30/17.
//

#include "Math/EctoMath.h"
#include "Math/DataTypes/Rotation2D.h"

//TODO Find out what all of these functions do, and comment them

double EctoMath::radiansToDegrees(double radians) {
	return (radians / M_PI) * 180.0;
}

double EctoMath::degreesToRadians(double degrees) {
	return (degrees / 180.0) * M_PI;
}

double EctoMath::addTwoAngles(double first, double second) {
	return wrapAngle(first + second);
}

double EctoMath::wrapAngle(double angle) {
	angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
	if (angle > M_PI) {
		angle -= 2 * M_PI;
	} else if (angle < -M_PI) {
		angle += 2 * M_PI;
	}
	return angle;
}

double EctoMath::shortestAngleTurn(double current, double target) {
	double angleToTurn = target - current;
	
	if (std::abs(angleToTurn) > M_PI) {
		if (current > 0) {
			if (angleToTurn > 0) {
				angleToTurn = angleToTurn - M_PI;
			} else {
				angleToTurn = angleToTurn + 2.0 * M_PI;
			}
		} else {
			if (angleToTurn > 0) {
				angleToTurn = angleToTurn - 2.0 * M_PI;
			} else {
				angleToTurn = angleToTurn + M_PI;
			}
		}
	}
	
	return angleToTurn;
}

Rotation2D EctoMath::convertTo360(const Rotation2D &input) {
	double radians = input.getRadians();
	
	return Rotation2D::fromRadians(radians + M_PI);
}

double EctoMath::sinc(double x) {
	if (x == 0)
		return 1;
	
	return std::sin(x) / x;
}

double EctoMath::cosc(double x) {
	if (x == 0)
		return 1;
	
	return std::cos(x) / x;
}

double EctoMath::map(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std::pair<double, double> EctoMath::polarToCartesian(double r, double th) {
    double x = r * std::cos(th);
    double y = r * std::sin(th);
    return {x, y};
}

double EctoMath::extrapolate(double in, double x1, double y1, double x2, double y2) {
    return y1 + ((in - x1) / (x2 -x1)) * (y2 - y1);
}

bool EctoMath::isPoseInRegion(frc::Pose2d pose, frc::Translation2d bottomLeftPoint,
                              frc::Translation2d topRightPoint) {
    return pose.X() >= bottomLeftPoint.X() &&
           pose.X() <= topRightPoint.X() &&
           pose.Y() >= bottomLeftPoint.Y() &&
           pose.Y() <= topRightPoint.Y();
}

bool EctoMath::epsilonEquals(double x, double y, double epsilon) {
    return (x - epsilon <= y) && (x + epsilon >= y);

}

bool EctoMath::epsilonEquals(double x, double y) {
    return epsilonEquals(x, y, 1e-12);
}

frc::Pose2d EctoMath::interpolate(frc::Pose2d current, frc::Pose2d forward, double t) {
    if (t < 0) return current;
    if (t >= 1) return forward;
    auto twist = current.Log(forward) * t;
    return current.Exp(twist);
}

double EctoMath::interpolate(double x1, double x2, double t) {
    return x1 + (x2 - x1) * std::clamp(t, 0.0, 1.0);
}

