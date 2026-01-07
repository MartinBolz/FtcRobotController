package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Field OpMode", group = "Drive")
public class MecanumFieldOpmode extends OpMode {
    MechanumDrive drive = new MechanumDrive();

    double forward, strafe,rotate;
    double leftStickPower, rightStickPower;

    @Override
    public void init(){
        drive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");



    }

    @Override
    public void loop(){
        /*
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        telemetry.addData("Input", "F:%.2f, S:%.2f, R:%.2f", forward, strafe, rotate);
        drive.drive(forward, strafe,rotate);
        //drive.driveFieldRelative(forward, strafe,rotate);
        */
        // --- TANK DRIVE CONTROLS ---

        // Read the vertical position of each joystick.
        // Add a negative sign because pushing the sticks forward results in a negative value.
        leftStickPower = -gamepad1.left_stick_y;
        rightStickPower = -gamepad1.right_stick_y;

        // Send the stick powers to our new tankDrive method.
        drive.tankDrive(leftStickPower, rightStickPower);

        // Display the power values on the Driver Hub for debugging.
        telemetry.addData("Left Power", "%.2f", leftStickPower);
        telemetry.addData("Right Power", "%.2f", rightStickPower);
        telemetry.update();

    }
}
