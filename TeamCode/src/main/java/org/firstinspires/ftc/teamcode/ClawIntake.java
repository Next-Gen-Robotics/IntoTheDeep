package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class ClawIntake extends LinearOpMode {

    public Servo clawservo;
    public static double SERVO_POSITION;
    public static boolean isLongEdge = true;  // Boolean to toggle between long and short edge collection

    // Positions for claw operation
    public static final double LONG_EDGE_SERVO_POS = 0.15; // Opening for long edge
    public static final double SHORT_EDGE_SERVO_POS = 0.2; // Opening for short edge
    public static final double COLLECT_POS = 0.25; // Closed position
    public static final double DROP_POS = 1.0; // Open position (if fully open required)

    private boolean aButtonPressed = false;  // Track if A button has been pressed

    @Override
    public void runOpMode() throws InterruptedException {

        clawservo = hardwareMap.get(Servo.class, "ClawIntake");

        // Set the initial position of the claw to closed (COLLECT_POS)
        clawservo.setPosition(COLLECT_POS);
        SERVO_POSITION = COLLECT_POS;  // Initially in the closed position

        // Display initializing status
        telemetry.addLine("Initializing with claw closed");
        telemetry.update();

        waitForStart();  // Wait for start of op mode

        while (opModeIsActive()) {

            // Toggle between long and short edge using 'A' button with debouncing logic
            if (gamepad1.a && !aButtonPressed) {
                isLongEdge = !isLongEdge;
                aButtonPressed = true;  // Button press registered
            } else if (!gamepad1.a) {
                aButtonPressed = false;  // Reset button press when released
            }

            // Set the claw position based on the edge type
            if (isLongEdge) {
                SERVO_POSITION = LONG_EDGE_SERVO_POS;
            } else {
                SERVO_POSITION = SHORT_EDGE_SERVO_POS;
            }

            // Collect the sample by pressing 'X' button
            if (gamepad1.x) {
                SERVO_POSITION = COLLECT_POS;  // Close claw to collect
            }

            // Drop the sample by pressing 'B' button
            if (gamepad1.b) {
                SERVO_POSITION = DROP_POS;  // Open claw to drop
            }

            // Apply the servo position
            if (SERVO_POSITION >= 0 && SERVO_POSITION <= 1) { // Ensure valid position range
                clawservo.setPosition(SERVO_POSITION);
            }

            // Telemetry for debugging and feedback
            telemetry.addData("Claw Position", SERVO_POSITION);
            telemetry.addData("Collecting Sample On", isLongEdge ? "Long Edge" : "Short Edge");
            telemetry.addLine(gamepad1.x ? "Collecting..." : gamepad1.b ? "Dropping..." : "Idle");
            telemetry.update();
        }
    }
}
