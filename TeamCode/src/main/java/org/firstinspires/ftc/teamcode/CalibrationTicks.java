package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Ticks", group="Calibration")
public class CalibrationTicks extends LinearOpMode {

    private DcMotorEx slideMotor;
    private DcMotor armMotor;
    private Servo wrist;

    private double slidePosition = 0;
    private double armPosition = 0;
    private double wristPosition = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Set initial positions
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calibration steps:
        telemetry.addData("Status", "Ready for Calibration");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Manual control for the slide using right stick Y
            double slideControl = -gamepad1.right_stick_y;
            slidePosition += slideControl * 20; // Adjust sensitivity as needed
            slidePosition = Range.clip(slidePosition, 0, 5000); // Change limits based on physical constraints
            slideMotor.setTargetPosition((int) slidePosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(0.5);

            // Manual control for the arm using left stick Y
            double armControl = -gamepad1.left_stick_y;
            armPosition += armControl * 20; // Adjust sensitivity as needed
            armPosition = Range.clip(armPosition, -1000, 1000); // Change limits based on physical constraints
            armMotor.setTargetPosition((int) armPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);

            // Manual control for the wrist using gamepad buttons
            if (gamepad1.a) {
                wristPosition += 0.01; // Increase wrist position
            } else if (gamepad1.b) {
                wristPosition -= 0.01; // Decrease wrist position
            }
            wristPosition = Range.clip(wristPosition, 0, 1); // Range for servos is typically between 0 and 1
            wrist.setPosition(wristPosition);

            // Display telemetry data for calibration
            telemetry.addData("Slide Target Position", slideMotor.getTargetPosition());
            telemetry.addData("Slide Current Position", slideMotor.getCurrentPosition());

            telemetry.addData("Arm Target Position", armMotor.getTargetPosition());
            telemetry.addData("Arm Current Position", armMotor.getCurrentPosition());

            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.update();
        }
    }
}