package org.firstinspires.ftc.teamcode.TestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Zero-er", group = "Tuning")
public class ZeroAxonPositions extends LinearOpMode {

    private Servo leftHoodServo;
    private Servo rightHoodServo;
    private AnalogInput leftServoEncoder;
    private AnalogInput rightServoEncoder;

    private static final double ZERO_POS = 0.0;

    @Override
    public void runOpMode() {

        leftHoodServo = hardwareMap.get(Servo.class, "axonPowerLeft");
        rightHoodServo = hardwareMap.get(Servo.class, "axonPowerRight");

        leftServoEncoder = hardwareMap.get(AnalogInput.class, "leftServoEncoder");
        rightServoEncoder = hardwareMap.get(AnalogInput.class, "rightServoEncoder");

        rightHoodServo.setDirection(Servo.Direction.REVERSE);

        // Hold both servos at zero during INIT
        while (opModeInInit()) {
            telemetry.addLine("INSTRUCTIONS:");
            telemetry.addLine("1) Remove servo gears");
            telemetry.addLine("2) Keep robot on and enter INIT");
            telemetry.addLine("3) Servos are being held at position 0.0");
            telemetry.addLine("4) Put the hood all the way down");
            telemetry.addLine("5) Install gears at this position");
            telemetry.addLine("6) Press START when finished");

            telemetry.addData("Left Servo Cmd", ZERO_POS);
            telemetry.addData("Right Servo Cmd", ZERO_POS);
            telemetry.addData("Left Encoder Voltage", "%.3f", leftServoEncoder.getVoltage());
            telemetry.addData("Right Encoder Voltage", "%.3f", rightServoEncoder.getVoltage());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Keep holding zero after Start too
        while (opModeIsActive()) {
            leftHoodServo.setPosition(ZERO_POS);
            rightHoodServo.setPosition(ZERO_POS);

            telemetry.addLine("Hood servos held at position 0.0");
            telemetry.addData("Left Servo Cmd", ZERO_POS);
            telemetry.addData("Right Servo Cmd", ZERO_POS);
            telemetry.addData("Left Encoder Voltage", "%.3f", leftServoEncoder.getVoltage());
            telemetry.addData("Right Encoder Voltage", "%.3f", rightServoEncoder.getVoltage());
            telemetry.update();
        }
    }
}