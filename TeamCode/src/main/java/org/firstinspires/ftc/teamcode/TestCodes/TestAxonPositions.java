package org.firstinspires.ftc.teamcode.TestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AXON Hood Down Tuner", group = "Tuning")
public class TestAxonPositions extends LinearOpMode {

    private Servo leftHoodServo;
    private Servo rightHoodServo;

    private AnalogInput leftServoEncoder;
    private AnalogInput rightServoEncoder;

    // This is the value you are tuning for NearRed.HOOD_DOWN_POS
    private double targetPosition = 0.50;

    // Smaller step for real tuning
    private static final double STEP = 0.01;

    // Encoder zero offsets for relative angle readout
    private double leftZeroOffset = 0.0;
    private double rightZeroOffset = 0.0;

    // Edge detection
    private boolean lastSquare = false;
    private boolean lastCircle = false;
    private boolean lastCross = false;

    @Override
    public void runOpMode() {
        leftHoodServo = hardwareMap.get(Servo.class, "axonPowerLeft");
        rightHoodServo = hardwareMap.get(Servo.class, "axonPowerRight");

        leftServoEncoder = hardwareMap.get(AnalogInput.class, "leftServoEncoder");
        rightServoEncoder = hardwareMap.get(AnalogInput.class, "rightServoEncoder");

        // Match your real robot setup
        rightHoodServo.setDirection(Servo.Direction.REVERSE);

        // Start at current target
        leftHoodServo.setPosition(targetPosition);
        rightHoodServo.setPosition(targetPosition);

        telemetry.addLine("AXON Hood Down Tuner");
        telemetry.addLine("Square = -0.01");
        telemetry.addLine("Circle = +0.01");
        telemetry.addLine("Cross = set encoder zero at current hood position");
        telemetry.addLine("Tune until hood is fully down and repeatable.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean square = gamepad2.square;
            boolean circle = gamepad2.circle;
            boolean cross = gamepad2.cross;

            // Set encoder zero at current physical hood angle
            if (cross && !lastCross) {
                leftZeroOffset = leftServoEncoder.getVoltage();
                rightZeroOffset = rightServoEncoder.getVoltage();
            }

            // Square -> decrease target position
            if (square && !lastSquare) {
                targetPosition = Range.clip(targetPosition - STEP, 0.0, 1.0);
            }

            // Circle -> increase target position
            if (circle && !lastCircle) {
                targetPosition = Range.clip(targetPosition + STEP, 0.0, 1.0);
            }

            // Always command current target
            leftHoodServo.setPosition(targetPosition);
            rightHoodServo.setPosition(targetPosition);

            lastSquare = square;
            lastCircle = circle;
            lastCross = cross;

            // Raw encoder voltages
            double leftVoltage = leftServoEncoder.getVoltage();
            double rightVoltage = rightServoEncoder.getVoltage();

            double leftMax = leftServoEncoder.getMaxVoltage();
            double rightMax = rightServoEncoder.getMaxVoltage();

            // Relative-to-zero voltage
            double leftAdjusted = leftVoltage - leftZeroOffset;
            double rightAdjusted = rightVoltage - rightZeroOffset;

            if (leftAdjusted < 0) leftAdjusted += leftMax;
            if (rightAdjusted < 0) rightAdjusted += rightMax;

            // Relative normalized values
            double leftNormalized = leftAdjusted / leftMax;
            double rightNormalized = rightAdjusted / rightMax;

            // Relative angles
            double leftDegrees = leftNormalized * 360.0;
            double rightDegrees = rightNormalized * 360.0;

            telemetry.addLine("=== HOOD DOWN TUNING ===");
            telemetry.addData("Target Servo Position", "%.3f", targetPosition);
            telemetry.addData("Copy into HOOD_DOWN_POS", "%.3f", targetPosition);

            telemetry.addLine();
            telemetry.addLine("=== LEFT ENCODER ===");
            telemetry.addData("Raw Voltage", "%.3f", leftVoltage);
            telemetry.addData("Relative Normalized", "%.3f", leftNormalized);
            telemetry.addData("Relative Degrees", "%.1f", leftDegrees);

            telemetry.addLine();
            telemetry.addLine("=== RIGHT ENCODER ===");
            telemetry.addData("Raw Voltage", "%.3f", rightVoltage);
            telemetry.addData("Relative Normalized", "%.3f", rightNormalized);
            telemetry.addData("Relative Degrees", "%.1f", rightDegrees);

            telemetry.addLine();
            telemetry.addData("Zero Offset Set", leftZeroOffset != 0.0 ? "YES" : "NO");
            telemetry.addData("Suggested NearRed line", "public static double HOOD_DOWN_POS = %.3f;", targetPosition);

            telemetry.update();
        }
    }
}