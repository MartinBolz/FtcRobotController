package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDrive {
    private DcMotor rr, rf, lr, lf;

    private IMU imu;

    public void init(HardwareMap hwMap){
        lf = hwMap.get(DcMotor.class,"lf");
        lr = hwMap.get(DcMotor.class,"lr");
        rf = hwMap.get(DcMotor.class, "rf");
        rr = hwMap.get(DcMotor.class,"rr");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, //logo direction
                RevHubOrientationOnRobot.UsbFacingDirection.UP); //usb direction

        imu.initialize((new IMU.Parameters(revOrientation)));

    }


    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        lf.setPower(maxSpeed * (frontLeftPower / maxPower));
        lr.setPower(maxSpeed * (backLeftPower / maxPower));
        rf.setPower(maxSpeed * (frontRightPower / maxPower));
        rr.setPower(maxSpeed * (backRightPower / maxPower));

    }

    public void tankDrive(double leftPower, double rightPower) {
        // Note: Because you reversed the left motors in init(),
        // sending a positive leftPower will make them go forward.
        lf.setPower(-leftPower);
        lr.setPower(leftPower);

        // Right motors are not reversed, so positive rightPower makes them go forward.
        rf.setPower(-rightPower);
        rr.setPower(rightPower);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward,newStrafe, rotate);

    }
}
