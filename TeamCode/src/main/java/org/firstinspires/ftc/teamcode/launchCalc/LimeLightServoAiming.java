package org.firstinspires.ftc.teamcode.launchCalc;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LimeLight Servo Aim", group = "Drive")
public class LimeLightServoAiming extends OpMode {

    private Limelight3A ll;
    private IMU imu;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 60.0;


    @Override
    public void init(){
        //limelight 3a setup
        ll = hardwareMap.get(Limelight3A.class,"camera");
        ll.pipelineSwitch(0); //pipeline for red team and blue team
        ll.start();


        //imu setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, //logo direction
                RevHubOrientationOnRobot.UsbFacingDirection.UP); //usb direction
        imu.initialize((new IMU.Parameters(revOrientation)));

    }



    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        ll.updateRobotOrientation(orientation.getYaw());

        //pull our beautiful data from LL!
        LLResult llResult = ll.getLatestResult();

        if (llResult != null && llResult.isValid()){
            double targetOffsetAngle_Vertical = llResult.getTy();

            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = 25.0;



            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            //Pose3D botposeMt2= llResult.getBotpose_MT2(); //MegaTag 2 returns

            //ll results not MT2
            telemetry.addData("Distance in inches", distanceFromLimelightToGoalInches);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());

            //target area
            telemetry.addData("Target Area", llResult.getTa());
            //telemetry.addData("Bot Pose", botposeMt2.toString());
            //telemetry.addData("Yaw", botposeMt2.getOrientation().getYaw());


        }

    }



}
