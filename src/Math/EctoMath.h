//
// Created by Abiel on 10/30/17.
//

#ifndef BOTBUSTERSREBIRTH_ECTOMATH_H
#define BOTBUSTERSREBIRTH_ECTOMATH_H

#include <cmath>
#include <bits/stdc++.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

using namespace std;

class Rotation2D;

namespace EctoMath {
	double radiansToDegrees(double radians);
	
	double degreesToRadians(double degrees);
	
	double addTwoAngles(double first, double second);
	
	double wrapAngle(double angle);
	
	double shortestAngleTurn(double current, double target);

    std::pair<double, double> polarToCartesian(double r, double th);
	
	Rotation2D convertTo360(const Rotation2D &input);
	
	double sinc(double x);
	
	double cosc(double x);

	double map(double x, double in_min, double in_max, double out_min, double out_max);

    double extrapolate(double in, double x1, double y1, double x2, double y2);

    bool isPoseInRegion(frc::Pose2d pose, frc::Translation2d bottomLeftPoint, frc::Translation2d topRightPoint);

    bool epsilonEquals(double x, double y, double epsilon);

    bool epsilonEquals(double x, double y);

    frc::Pose2d interpolate(frc::Pose2d current, frc::Pose2d forward, double t);

    double interpolate(double x1, double x2, double t);
}

#endif