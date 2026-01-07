package org.firstinspires.ftc.teamcode.launchCalc;

import org.firstinspires.ftc.teamcode.launchCalc.ballistics;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight Distance", group = "Drive")
public class LimeLightDistance extends OpMode {

    private Limelight3A ll;
    private IMU imu;

    private ballistics ballistics;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the target to the floor
    double goalHeightInches = 60.0;

    //the distance between robot and the qr code
    //takes in the LLResult from LL3A
    double xDistanceInches(LLResult llResult){
        double targetOffsetAngle_Vertical = llResult.getTy();
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        double distanceFromLimelightToGoalCm = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);

        double distance = distanceFromLimelightToGoalCm * 0.393701;

        return distance;
    }



    @Override
    public void init() {
        // limelight 3a setup
        ll = hardwareMap.get(Limelight3A.class, "camera");
        ll.pipelineSwitch(0); // pipeline for red team and blue team
        ll.start();

    }


    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        ll.updateRobotOrientation(orientation.getYaw());

        // pull our beautiful data from LL!
        LLResult llResult = ll.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double distance_inches = xDistanceInches(llResult);
            double distanceFromLimelightToGoalCm = distance_inches / 0.393701;

            // ll results
            telemetry.addData("Distance in cm", distanceFromLimelightToGoalCm);
            telemetry.addData("Distance in inches", distance_inches);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());

            // target area
            telemetry.addData("Target Area", llResult.getTa());

            ballistics.ballisticsStats(telemetry, xDistanceInches(llResult), 0.0);
            ballistics.robotStats(telemetry);

        }
    }











        /* og stuff with megaTag2


        @Override
    public void init() {
        // limelight 3a setup
        ll = hardwareMap.get(Limelight3A.class, "camera");
        ll.pipelineSwitch(0); // pipeline for red team and blue team
        ll.start();

        /*
         * //fieldmap shit for MT2
         * LLFieldMap fieldMap = new LLFieldMap(); // You'll need to fill this with
         * field data
         * boolean success = ll.uploadFieldmap(fieldMap, null); // null means use the
         * default slot
         * if (success) {
         * telemetry.addData("Field Map", "Uploaded successfully!");
         * } else {
         * telemetry.addData("Field Map", "Oops, upload failed");
         * }
         *

        // imu setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, // logo direction
                RevHubOrientationOnRobot.UsbFacingDirection.UP); // usb direction
        imu.initialize((new IMU.Parameters(revOrientation)));

    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        ll.updateRobotOrientation(orientation.getYaw());

        // pull our beautiful data from LL!
        LLResult llResult = ll.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double distance_inches = xDistance(llResult);
            double distanceFromLimelightToGoalCm = distance_inches / 0.393701;

            // Pose3D botposeMt2= llResult.getBotpose_MT2(); //MegaTag 2 returns

            // ll results not MT2
            telemetry.addData("Distance in cm", distanceFromLimelightToGoalCm);
            telemetry.addData("Distance in inches", distance_inches);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());

            // target area
            telemetry.addData("Target Area", llResult.getTa());
            // telemetry.addData("Bot Pose", botposeMt2.toString());
            // telemetry.addData("Yaw", botposeMt2.getOrientation().getYaw());

        }


        /*
         * //fiduciary result
         * List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
         * for (LLResultTypes.FiducialResult fiducial : fiducials) {
         * int id = fiducial.getFiducialId(); // The ID number of the fiducial
         * double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
         * double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
         * double StrafeDistance_3D =
         * fiducial.getRobotPoseTargetSpace().getPosition().y;
         * 
         * double distance =
         * Math.abs(fiducial.getRobotPoseTargetSpace().getPosition().z);;
         * 
         * telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
         * }
         * 
         * //fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag
         * Coordinate System (Most Useful)
         * //fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the
         * AprilTag (useful)
         * //fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate
         * system based on this tag alone (useful)
         * //fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's
         * coordinate system (not very useful)
         * //fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's
         * coordinate system (not very useful)
         */

}
