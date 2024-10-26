package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="InToTheDeepAuto", group="Robot")
public class InToTheDeepAuto extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx slideMotor = null;
    private DcMotor armMotor = null;
    private Servo wrist = null;
    private Servo claw = null;
    private IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.09;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

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
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set direction and behavior for motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        initializeEncoders();

        // Initialize IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        if (opModeIsActive()) {
            claw.setPosition(0.7030);
            sleep(500);

            moveArm(50, 0.5);
            moveSlide(300, 0.5);
            wrist.setPosition(0.3);
            //sleep(500);

            encoderDriveWithIMU(DRIVE_SPEED, 10, 0, 5.0); // Move forward 24 inches with heading correction
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void encoderDriveWithIMU(double speed, double distanceInches, double targetHeading, double timeoutS) {
        int targetPosition = (int) (distanceInches * COUNTS_PER_INCH);
        resetDriveEncoders();

        // Set target positions for encoders
        frontLeftDrive.setTargetPosition(targetPosition);
        frontRightDrive.setTargetPosition(targetPosition);
        backLeftDrive.setTargetPosition(targetPosition);
        backRightDrive.setTargetPosition(targetPosition);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && driveIsBusy()) {
            // Get the current heading from the IMU
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate heading correction
            double headingError = targetHeading - currentHeading;
            double correction = headingError * 0.1;  // Adjust this gain as needed

            // Adjust motor power for heading correction
            double leftPower = speed + correction;
            double rightPower = speed - correction;

            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

            telemetry.addData("Target", "%7d", targetPosition);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        setDrivetrainPower(0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Reset mode after driving
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeftDrive.setZeroPowerBehavior(behavior);
        frontRightDrive.setZeroPowerBehavior(behavior);
        backLeftDrive.setZeroPowerBehavior(behavior);
        backRightDrive.setZeroPowerBehavior(behavior);
        slideMotor.setZeroPowerBehavior(behavior);
        armMotor.setZeroPowerBehavior(behavior);
    }

    private void initializeEncoders() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveArm(double degrees, double power) {
        int targetPosition = (int) (degrees * ARM_TICKS_PER_DEGREE);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);

        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setPower(0);
    }

    private void moveSlide(double mm, double power) {
        int targetPosition = (int) (mm * LIFT_TICKS_PER_MM);
        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(power);

        while (opModeIsActive() && slideMotor.isBusy()) {
            telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
        slideMotor.setPower(0);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    private void resetDriveEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean driveIsBusy() {
        return frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy();
    }

    private void setDrivetrainPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }
}