package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="LeagueOne", group="Robot")
public class LeagueOne extends LinearOpMode {

    // Declare OpMode members
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx slideMotor = null;
    private DcMotor armMotor = null;
    private Servo wrist = null;
    private Servo claw = null;  // Replacing intake wheel with claw mechanism

    // Constants for arm and lift movements
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    // Variables for tracking positions
    private double armPositionDegrees = 0; // Using degrees for arm position
    private double slidePositionMM = 0;    // Using millimeters for slide position
    private double wristPosition = 0;      // Wrist servo position, no initialization

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw"); // Initialize the claw servo

        // Set motor directions for drivetrain
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to brake when zero power is applied
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial positions for arm and slide
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game driver to press play
        waitForStart();

        // Main OpMode loop
        while (opModeIsActive()) {
            // Drivetrain controls
            double y = gamepad1.left_stick_y; // Forward/backward
            double x = - gamepad1.left_stick_x;  // Left/right strafing
            double rx = gamepad1.right_stick_x; // Rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            // Slide control using bumpers
            if (gamepad1.right_bumper) {
                slidePositionMM += 5; // Raise slide
            } else if (gamepad1.left_bumper) {
                slidePositionMM -= 5; // Lower slide
            }
            slidePositionMM = Range.clip(slidePositionMM, 0, 600); // Limit slide position
            int slideTargetPosition = (int) (slidePositionMM * LIFT_TICKS_PER_MM); // Convert mm to ticks
            slideMotor.setTargetPosition(slideTargetPosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(0.5);

            // Arm control using D-pad up/down
            if (gamepad1.dpad_up) {
                armPositionDegrees += 2; // Raise arm
            } else if (gamepad1.dpad_down) {
                armPositionDegrees -= 2; // Lower arm
            }
            armPositionDegrees = Range.clip(armPositionDegrees, 0, 145); // Limit arm position
            int armTargetPosition = (int) (armPositionDegrees * ARM_TICKS_PER_DEGREE); // Convert degrees to ticks
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);

            // Wrist control using D-pad left/right with torque-based servo handling
            if (gamepad1.dpad_right) {
                if (wristPosition == 0) wristPosition = wrist.getPosition(); // Initialize wristPosition from the current servo position
                wristPosition = Math.max(0, wristPosition - 0.01); // Move wrist in
            } else if (gamepad1.dpad_left) {
                if (wristPosition == 0) wristPosition = wrist.getPosition(); // Initialize wristPosition from the current servo position
                wristPosition = Math.min(1, wristPosition + 0.01); // Move wrist out
            }
            if (wristPosition != 0) {
                wrist.setPosition(wristPosition); // Only set position if wristPosition has been updated
            }

            // Claw control using A (close), X (stop), and B (open)
            if (gamepad1.b) {
                claw.setPosition(0.9368);  // Open the claw
            } else if (gamepad1.a) {
                claw.setPosition(0.7030);  // Close the claw
            } else if (gamepad1.x) {
                claw.setPosition(0.10);  // Stop claw movement (neutral)
            }

            // Telemetry for debugging and calibration
            telemetry.addData("Slide Position (mm):", slidePositionMM);
            telemetry.addData("Slide Target Position (ticks):", slideMotor.getTargetPosition());
            telemetry.addData("Slide Current Position (ticks):", slideMotor.getCurrentPosition());

            telemetry.addData("Arm Position (degrees):", armPositionDegrees);
            telemetry.addData("Arm Target Position (ticks):", armMotor.getTargetPosition());
            telemetry.addData("Arm Current Position (ticks):", armMotor.getCurrentPosition());

            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());  // Added claw telemetry
            telemetry.update();
        }
    }
}