package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.util.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.util.FieldMath;
import org.firstinspires.ftc.teamcode.subsystem.util.HoodLookupTable;
import org.firstinspires.ftc.teamcode.subsystem.util.PoseStorage;

@TeleOp
public class NewPremierDrive extends LinearOpMode {

    private HoodLookupTable hoodLookup;
    private TelemetryManager robotTelemetry;

    private PIDFController flywheelController;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;

    public static double SQUARE_TARGET_VELOCITY = 1200.0;
    public static double CIRCLE_TARGET_VELOCITY = 1500.0;

    private VoltageSensor voltageSensor;
    public static double voltageControllerWasTunedAt = 12.3; //TODO: Change

    MecanumDrive drive;
    DcMotor encoder;
    DcMotor transferMotor;
    DcMotor rightFrontMotor, rightBackMotor, leftBackMotor, leftFrontMotor;
    DcMotorEx leftHoodMotor, rightHoodMotor;
    DcMotor intakeMotor;
    Servo axonPowerLeft, axonPowerRight;



    double REALFLYWHEELSPEED;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        HoodLookupTable hood = new HoodLookupTable();

        flywheelController = new PIDFController(P, I, D, F, kV, kA, kS);
        flywheelController.reset();

        robotTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        telemetry.addData("Stored X: ", PoseStorage.currentPose.position.x - 12);
        telemetry.addData("Stored Y: ", PoseStorage.currentPose.position.y);
        telemetry.addData("Stored Heading: ", Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
        telemetry.update();

        FieldMath.Alliance alliance = FieldMath.Alliance.RED; //TODO: Change to Red/Blue

        double SpeedFactor;
        double flyWheelAcceleration;

        axonPowerLeft = hardwareMap.get(Servo.class, "axonPowerLeft");
        axonPowerRight = hardwareMap.get(Servo.class, "axonPowerRight");

        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");

        leftHoodMotor = hardwareMap.get(DcMotorEx.class, "leftHoodMotor");
        rightHoodMotor = hardwareMap.get(DcMotorEx.class, "rightHoodMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        voltageSensor = hardwareMap.voltageSensor.get("Control Hub");

        SpeedFactor = 1.0;
        flyWheelAcceleration = 0.06;

        axonPowerLeft.setDirection(Servo.Direction.FORWARD);
        axonPowerRight.setDirection(Servo.Direction.REVERSE);

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftHoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d livePose = drive.localizer.getPose();

            double distanceToDepot = FieldMath.distanceToDepot(
                    livePose,
                    -12.0,
                    alliance
            );

            double hoodPos = hood.getServoPosition(distanceToDepot);

            axonPowerLeft.setPosition(hoodPos);
            axonPowerRight.setPosition(hoodPos);

            // robotTelemetry.

                /*
                * // FIELD-CENTRIC DRIVE
    double y = -gamepad1.left_stick_y;      // forward/back
    double x = gamepad1.left_stick_x * 1.1; // strafe, with small correction
    double rx = gamepad1.right_stick_x;     // rotation

    //Use Road Runner / localizer heading
    double botHeading = livePose.heading.toDouble();

    // Rotate the joystick vector counter to robot heading
    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

    // Normalize so powers keep the same ratio
    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    double frontLeftPower  = (rotY + rotX + rx) / denominator;
    double backLeftPower   = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower  = (rotY + rotX - rx) / denominator;

    leftFrontMotor.setPower(frontLeftPower * SpeedFactor);
    leftBackMotor.setPower(backLeftPower * SpeedFactor);
    rightFrontMotor.setPower(frontRightPower * SpeedFactor);
    rightBackMotor.setPower(backRightPower * SpeedFactor);
                *
                * */


            leftBackMotor.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * SpeedFactor);
            leftFrontMotor.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedFactor);
            rightBackMotor.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * SpeedFactor);
            rightFrontMotor.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * SpeedFactor);

            double targetVelocity = 0.0;

            if (gamepad2.left_trigger > 0.1) {
                leftHoodMotor.setPower(1.0);
                rightHoodMotor.setPower(1.0);
                flywheelController.reset();

            } else if (gamepad2.left_bumper) {
                leftHoodMotor.setPower(-1.0);
                rightHoodMotor.setPower(-1.0);
                flywheelController.reset();

            } else {
                if (gamepad2.square) {
                    targetVelocity = SQUARE_TARGET_VELOCITY;
                    VibrateControllerBasedOnSpeed((int) SQUARE_TARGET_VELOCITY);
                } else if (gamepad2.circle) {
                    targetVelocity = CIRCLE_TARGET_VELOCITY;
                    VibrateControllerBasedOnSpeed((int) CIRCLE_TARGET_VELOCITY);
                }

                if (targetVelocity > 0.0) {
                    double currentSystemVoltage = voltageSensor.getVoltage();

                    flywheelController.setPIDF(P, I, D, F);
                    flywheelController.setFeedforward(kV, kA, kS);

                    REALFLYWHEELSPEED = Math.abs(rightHoodMotor.getVelocity());

                    double power = flywheelController.calculate(
                            targetVelocity - REALFLYWHEELSPEED,
                            targetVelocity,
                            0.0
                    ) * (voltageControllerWasTunedAt / currentSystemVoltage);

                    leftHoodMotor.setPower(power);
                    rightHoodMotor.setPower(power);
                } else {
                    leftHoodMotor.setPower(0.0);
                    rightHoodMotor.setPower(0.0);
                    flywheelController.reset();
                }
            }

            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(1);
            } else if (gamepad1.right_bumper) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.square) {
                transferMotor.setPower(1);
            } else if (gamepad1.circle || gamepad2.a) {
                transferMotor.setPower(-1);
            } else if (gamepad1.triangle) {
                transferMotor.setPower(0.5);
                intakeMotor.setPower(-1);
            } else {
                transferMotor.setPower(0);
            }

            REALFLYWHEELSPEED = Math.abs(rightHoodMotor.getVelocity());

            telemetry.addData("Flywheel Speed: ", REALFLYWHEELSPEED);
            telemetry.addData("Distance to Depot: ", distanceToDepot);
            telemetry.addData("Hood Servo Position: ", hoodPos);
            telemetry.addData("Live X: ", livePose.position.x);
            telemetry.addData("Live Y: ", livePose.position.y);
            telemetry.addData("Live Heading: ", Math.toDegrees(livePose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addData("Target Velocity: ", targetVelocity);
            telemetry.addData("Actual Velocity: ", REALFLYWHEELSPEED);
            telemetry.update();
        }
    }

    private void VibrateControllerBasedOnSpeed(int speedWanted) {
        if (Math.abs(REALFLYWHEELSPEED - speedWanted) < 200) {
            gamepad1.rumble(20);
            gamepad2.rumble(20);
        }
    }
}