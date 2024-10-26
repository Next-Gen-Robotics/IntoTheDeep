package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="NextGen Slide", group="Robot")
//@Disabled
public class IntoTheDeepSlide extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  frontLeftDrive   = null; //the left drivetrain motor
    public DcMotor  frontRightDrive  = null; //the right drivetrain motor
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotorEx slideMotor      = null; // Viper slide motor
    public DcMotor  hangMotor        = null; // Hanging motor remains the same
    public CRServo  intake           = null; //the active intake servo
    public Servo    wrist            = null; //the wrist servo

    /* This constant is the number of encoder ticks for each degree of rotation of the arm. */
    final double ARM_TICKS_PER_DEGREE =
            28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 214 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 126 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;

    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    final double WRIST_FOLDED_IN   = 0.52;
    final double WRIST_FOLDED_OUT  = 0.84;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 600 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;

    @Override
    public void runOpMode() {
        /* Define and Initialize Motors */
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor        = hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor      = hardwareMap.get(DcMotorEx.class, "slideMotor"); // Updated to slideMotor
        hangMotor       = hardwareMap.dcMotor.get("hangMotor");

        /* Setting directions and brake behavior */
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        /* Define and initialize servos. */
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Initialize IMU for field-centric control */
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        /* Wait for the game driver to press play */
        waitForStart();

        /* Main OpMode loop */
        while (opModeIsActive()) {
            // Drive controls
            double y = -gamepad1.left_stick_y; // Inverted y-axis for forward control
            double x = gamepad1.left_stick_x;  // Strafing left/right
            double rx = gamepad1.right_stick_x; // Rotating the robot

            // Reset IMU orientation when options button is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Adjust for bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator to keep motor power in range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            // Intake control
            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.y) {
                intake.setPower(INTAKE_DEPOSIT);
            }

            // Arm position control using buttons
            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            if (gamepad1.a) {
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.b) {
                armPosition = ARM_CLEAR_BARRIER;
            } else if (gamepad1.x) {
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            } else if (gamepad1.dpad_left) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_right) {
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_up) {
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_down) {
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            if (armPosition < 45 * ARM_TICKS_PER_DEGREE) {
                armLiftComp = (0.25568 * liftPosition);
            } else {
                armLiftComp = 0;
            }

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Slide motor control
            if (gamepad2.right_bumper) {
                liftPosition += 2800 * cycletime;
            } else if (gamepad2.left_bumper) {
                liftPosition -= 2800 * cycletime;
            }

            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET) {
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            if (liftPosition < 0) {
                liftPosition = 0;
            }

            slideMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) slideMotor).setVelocity(2100);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Hang motor control
            hangMotor.setPower(-gamepad2.left_stick_y);

            // Timing for cycle time calculation
            looptime = getRuntime();
            cycletime = looptime - oldtime;
            oldtime = looptime;

            // Send telemetry to display arm and lift positions
            telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position", slideMotor.getTargetPosition());
            telemetry.addData("lift current position", slideMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:", ((DcMotorEx) slideMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}