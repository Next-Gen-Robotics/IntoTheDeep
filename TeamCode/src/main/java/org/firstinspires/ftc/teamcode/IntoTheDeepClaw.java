package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="NextGenClaw", group="Robot")
public class IntoTheDeepClaw extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor armMotor = null;
    private Servo clawServo = null;
    private Servo wrist = null;

    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    // Positions for claw operation
    public static final double LONG_EDGE_SERVO_POS = 0.15; // Opening for long edge
    public static final double SHORT_EDGE_SERVO_POS = 0.2; // Opening for short edge
    public static final double COLLECT_POS = 0.25; // Closed position
    public static final double DROP_POS = 1.0; // Open position (if fully open required)

    private boolean isLongEdge = true;  // Boolean to toggle between long and short edge collection
    private boolean aButtonPressed = false;  // Track if A button has been pressed
    private double clawPosition = COLLECT_POS;  // Initially in closed position

    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor        = hardwareMap.get(DcMotor.class, "armMotor");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        // Initialize arm motor
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize servos
        clawServo = hardwareMap.get(Servo.class, "ClawIntake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        clawServo.setPosition(COLLECT_POS);
        wrist.setPosition(WRIST_FOLDED_IN);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            double forward = gamepad1.left_stick_y;
            double strafe = - gamepad1.left_stick_x;
            double rotate = - gamepad1.right_stick_x;

            // Scaling factor to reduce speed
            double speedScale = 0.5;

            // Calculate motor powers with scaling
            double frontLeftPower = (forward + strafe + rotate) * speedScale;
            double frontRightPower = (forward - strafe - rotate) * speedScale;
            double backLeftPower = (forward - strafe + rotate) * speedScale;
            double backRightPower = (forward + strafe - rotate) * speedScale;

            // Normalize the motor powers so none exceed 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Precision mode: reduce speed when left trigger is pressed
            double precisionScale = gamepad1.left_trigger > 0.1 ? 0.3 : 1.0;

            // Apply precision scaling to motor powers
            frontLeftPower *= precisionScale;
            frontRightPower *= precisionScale;
            backLeftPower *= precisionScale;
            backRightPower *= precisionScale;

            // Set motor powers
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Arm control logic
            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            // Claw intake control logic
            if (gamepad1.a && !aButtonPressed) {
                isLongEdge = !isLongEdge;
                aButtonPressed = true;  // Button press registered
            } else if (!gamepad1.a) {
                aButtonPressed = false;  // Reset button press when released
            }

            // Set the claw position based on the edge type
            if (isLongEdge) {
                clawPosition = LONG_EDGE_SERVO_POS;
            } else {
                clawPosition = SHORT_EDGE_SERVO_POS;
            }

            // Collect the sample by pressing 'X' button
            if (gamepad1.x) {
                clawPosition = COLLECT_POS;  // Close claw to collect
            }

            // Drop the sample by pressing 'B' button
            if (gamepad1.b) {
                clawPosition = DROP_POS;  // Open claw to drop
            }

            // Apply the servo position for the claw
            if (clawPosition >= 0 && clawPosition <= 1) { // Ensure valid position range
                clawServo.setPosition(clawPosition);
            }

            // Arm position adjustments and other control logic
            if(gamepad1.right_bumper) {
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                clawPosition = COLLECT_POS;
            } else if (gamepad1.left_bumper) {
                armPosition = ARM_CLEAR_BARRIER;
            } else if (gamepad1.y) {
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            } else if (gamepad1.dpad_left) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                clawPosition = COLLECT_POS;  // Close claw
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_right) {
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_up) {
                armPosition = ARM_ATTACH_HANGING_HOOK;
                clawPosition = COLLECT_POS;  // Keep claw closed
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_down) {
                armPosition = ARM_WINCH_ROBOT;
                clawPosition = COLLECT_POS;  // Keep claw closed
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Telemetry
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }
    }
}