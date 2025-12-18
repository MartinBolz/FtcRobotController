package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimeLightTesting extends OpMode {

    private Limelight3A ll;
    private IMU imu;

    @Override
    public void init(){
        //limelight 3a setup
        ll = hardwareMap.get(Limelight3A.class,"camera");
        ll.pipelineSwitch(0); //pipeline for red team and blue team
        ll.start();

        //fieldmap shit for MT2
        LLFieldMap fieldMap = new LLFieldMap(); // You'll need to fill this with field data
        boolean success = ll.uploadFieldmap(fieldMap, null); // null means use the default slot
        if (success) {
            telemetry.addData("Field Map", "Uploaded successfully!");
        } else {
            telemetry.addData("Field Map", "Oops, upload failed");
        }


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
        LLResult llResult = ll.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botposeMt2= llResult.getBotpose_MT2(); //MegaTag 2 returns

            //ll results not MT2
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());

            //target area
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Bot Pose", botposeMt2.toString());
            telemetry.addData("Yaw", botposeMt2.getOrientation().getYaw());


        }

        //fiduciary result
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            double StrafeDistance_3D =  fiducial.getRobotPoseTargetSpace().getPosition().y;

            double distance = Math.abs(fiducial.getRobotPoseTargetSpace().getPosition().z);;

            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
        }

        //fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
        //fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
        //fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
        //fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful)
        //fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful)



    }



}
