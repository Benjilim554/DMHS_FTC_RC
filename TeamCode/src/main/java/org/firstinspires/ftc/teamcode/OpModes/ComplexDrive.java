package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.subsystem.util.FieldMath;
import org.firstinspires.ftc.teamcode.subsystem.util.HoodLookupTable;
import org.firstinspires.ftc.teamcode.subsystem.util.PoseStorage;

@TeleOp
@Config
public class ComplexDrive extends LinearOpMode {

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

    double targetHeading = 0;
    boolean headingLock = false;

    public static double kHeadingP = 0.1; //TODO: tune this
    public static double TARGET_VELOCITY = 1200.0;

    MecanumDrive drive;
    DcMotor encoder;
    DcMotor transferMotor;
    DcMotor rightFrontMotor, rightBackMotor, leftBackMotor, leftFrontMotor;
    DcMotor leftHoodMotor, rightHoodMotor;
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

        leftHoodMotor = hardwareMap.get(DcMotor.class, "leftHoodMotor");
        rightHoodMotor = hardwareMap.get(DcMotor.class, "rightHoodMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

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

            boolean aligning = false;

            if (gamepad1.dpad_up) {
                targetHeading = Math.toRadians(126);
                aligning = true;
            }
            else if (gamepad1.dpad_right) {
                targetHeading = Math.toRadians(140);
                aligning = true;
            }
            else if (gamepad1.dpad_left) {
                targetHeading = Math.toRadians(-126);
                aligning = true;
            }
            else if (gamepad1.dpad_down) {
                targetHeading = Math.toRadians(-140);
                aligning = true;
            }

            double rx;

            if (aligning) {
                double currentHeading = livePose.heading.toDouble();
                double error = angleWrap(targetHeading - currentHeading);

                if (Math.abs(error) < Math.toRadians(2)) {
                    rx = 0;
                } else {
                    rx = error * kHeadingP;
                    rx = Math.max(-1.0, Math.min(1.0, rx));
                }
            } else {
                rx = gamepad1.right_stick_x;
            }

            leftBackMotor.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) + rx) * SpeedFactor);
            leftFrontMotor.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + rx) * SpeedFactor);
            rightBackMotor.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) - rx) * SpeedFactor);
            rightFrontMotor.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) - rx) * SpeedFactor);


            if (gamepad2.left_trigger > 0.1) {
                leftHoodMotor.setPower(1);
                rightHoodMotor.setPower(1);
            } else if (gamepad2.left_bumper) {
                leftHoodMotor.setPower(-1);
                rightHoodMotor.setPower(-1);
            } else if (gamepad2.square) {
                VibrateControllerBasedOnSpeed(1500);
                if (REALFLYWHEELSPEED < 1500) {
                    leftHoodMotor.setPower(1);
                    rightHoodMotor.setPower(1);
                } else {
                    leftHoodMotor.setPower(0);
                    rightHoodMotor.setPower(0);
                }
            } else if (gamepad2.circle) {
                VibrateControllerBasedOnSpeed(1200);
                if (REALFLYWHEELSPEED < 1200) {
                    leftHoodMotor.setPower(1);
                    rightHoodMotor.setPower(1);
                } else {
                    leftHoodMotor.setPower(0);
                    rightHoodMotor.setPower(0);
                }
            } else {
                leftHoodMotor.setPower(0);
                rightHoodMotor.setPower(0);
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

            REALFLYWHEELSPEED = Math.abs(((DcMotorEx) rightHoodMotor).getVelocity());

            telemetry.addData("Flywheel Speed: ", REALFLYWHEELSPEED);
            telemetry.addData("Distance to Depot: ", distanceToDepot);
            telemetry.addData("Hood Servo Position: ", hoodPos);
            telemetry.addData("Live X: ", livePose.position.x);
            telemetry.addData("Live Y: ", livePose.position.y);
            telemetry.addData("Live Heading: ", Math.toDegrees(livePose.heading.toDouble()));
            telemetry.update();
        }
    }

    private void VibrateControllerBasedOnSpeed(int speedWanted) {
        if (Math.abs(REALFLYWHEELSPEED - speedWanted) < 200) {
            gamepad1.rumble(20);
            gamepad2.rumble(20);
        }
    }
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

