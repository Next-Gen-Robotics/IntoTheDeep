package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class ClawCalibration extends LinearOpMode {

    public Servo clawservo;
    public static double SERVO_POSITION = 0.5; // Start with a neutral position

    @Override
    public void runOpMode() throws InterruptedException {

        clawservo = hardwareMap.get(Servo.class, "ClawIntake");

        telemetry.addLine("Calibrating...");
        telemetry.update();

        // Wait for the start of the opmode
        waitForStart();

        while (opModeIsActive()) {

            // Increase position with gamepad dpad-up
            if (gamepad1.dpad_up && SERVO_POSITION < 1.0) {
                SERVO_POSITION += 0.01; // Adjust in small increments
                sleep(100);  // Add a slight delay to prevent rapid adjustment
            }

            // Decrease position with gamepad dpad-down
            if (gamepad1.dpad_down && SERVO_POSITION > 0.0) {
                SERVO_POSITION -= 0.01; // Adjust in small increments
                sleep(100);  // Add a slight delay to prevent rapid adjustment
            }

            // Ensure the servo position stays within valid range (0 to 1)
            SERVO_POSITION = Math.max(0, Math.min(SERVO_POSITION, 1));

            // Apply the new servo position
            clawservo.setPosition(SERVO_POSITION);

            // Telemetry for feedback
            telemetry.addData("Claw Position", SERVO_POSITION);
            telemetry.update();
        }
    }
}
