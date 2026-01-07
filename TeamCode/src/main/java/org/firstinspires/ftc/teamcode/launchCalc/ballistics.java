package org.firstinspires.ftc.teamcode.launchCalc;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ballistics {
    double g = 9.81;
    double launcherHeight = 0.38; // meters want inches
    double launcherAngle = 40; // degrees
    double launcherAngleRad = Math.toRadians(launcherAngle);
    double launcherCos = Math.cos(launcherAngleRad);
    double launcherTan = Math.tan(launcherAngleRad);
    double wheelRadius = 0.0375; // meters
    double kSlip = 1.0; // 1.0 = 100% efficiency
    double targetHeight = 1.22; // meters want inches
    double goalDiff = targetHeight - launcherHeight;

    // this returns the height of the projectile at a given x position
    double projectile_height_given_x(double v, double x) {
        return x * launcherTan - (g * x * x) /
                (2 * v * v * launcherCos * launcherCos) + launcherHeight;
    }

    // returns the velocity needed to reach distance x
    double required_launch_velocity(double x) {
        double a = g * x * x;
        double b = 2 * launcherCos * launcherCos * (x * launcherTan - goalDiff);

        if (b < 0) {
            return -1;
        }

        return Math.sqrt(a / b);
    }

    // returns the slope of the projectile at distance x
    double projectile_slope_given_x(double v, double x) {
        // Derivative of trajectory equation: y' = tan(theta) - (g * x) / (v^2 *
        // cos^2(theta))
        return launcherTan - (g * x) / (v * v * launcherCos * launcherCos);
    }

    // converts ball velocity to wheel rpm
    double velocity2rpm(double vBall, double wheelRadius, double kSlip) {
        // vBall = kSlip * vWheel
        if (kSlip == 0)
            return 0;
        double vWheel = vBall / kSlip;
        return (vWheel * 60) / (2 * Math.PI * wheelRadius);
    }

    void robotStats(Telemetry telemetry) {

        telemetry.addData("launcherHeight", launcherHeight);
        telemetry.addData("launcherAngle", launcherAngle);
        telemetry.addData("wheelRadius", wheelRadius);
        telemetry.addData("targetHeight", targetHeight);
    }

    void ballisticsStats(Telemetry telemetry, double xDistance, double kSlip) {
        telemetry.addData("kSlip", kSlip);
        telemetry.addData("vBall: ", required_launch_velocity(xDistance));
        telemetry.addData("WheelRPM: ", velocity2rpm(required_launch_velocity(xDistance), wheelRadius, kSlip));
        telemetry.addData("Height: ", projectile_height_given_x(required_launch_velocity(xDistance), xDistance));
        telemetry.addData("Slope: ", projectile_slope_given_x(required_launch_velocity(xDistance), xDistance));
    }
}
