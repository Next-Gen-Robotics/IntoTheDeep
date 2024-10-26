package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Drivetrain Example", group="Robot")
public class OutReach extends LinearOpMode {

    public DcMotor leftDrive = null; // the left drivetrain motor
    public DcMotor rightDrive = null; // the right drivetrain motor

    @Override
    public void runOpMode() {
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); // the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // the right drivetrain motor

        // Set the direction for the motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to BRAKE for better control






































        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        // Wait for the game driver to press play
        waitForStart();

        // Run until the driver presses stop
        while (opModeIsActive()) {

            // Control forward movement and rotation using gamepad
            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;

            // Mix the forward and rotate values to determine motor power
            left = forward + rotate;
            right = forward - rotate;

            // Normalize the values so neither exceeds +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Set the motor power to the drivetrain motors
            leftDrive.setPower(left);
            rightDrive.setPower(right);
        }
    }
}
