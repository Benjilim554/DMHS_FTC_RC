package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.field.CanvasRotation;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystem.util.FieldMath;
import org.firstinspires.ftc.teamcode.subsystem.util.HoodLookupTable;
import org.firstinspires.ftc.teamcode.subsystem.util.PoseStorage;

@TeleOp
public class PremierDrive extends LinearOpMode {

    private HoodLookupTable hoodLookup;
    private TelemetryManager robotTelemetry;
    MecanumDrive drive;
    DcMotor encoder;
    CRServo transferServo;
    DcMotor rightFrontMotor, rightBackMotor, leftBackMotor, leftFrontMotor;
    DcMotor leftHoodMotor, rightHoodMotor;
    DcMotor intakeMotor;
    Servo axonPowerLeft, axonPowerRight;



    double REALFLYWHEELSPEED;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        HoodLookupTable hood = new HoodLookupTable();

        robotTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

     //   robotTelemetry.setOffsets(72.0, 72.0, CanvasRotation.DEG_0);

        telemetry.addData("Stored X: ", PoseStorage.currentPose.position.x - 12);
        telemetry.addData("Stored Y: ", PoseStorage.currentPose.position.y);
        telemetry.addData("Stored Heading: ", Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
        telemetry.update();

        FieldMath.Alliance alliance = FieldMath.Alliance.RED; //TODO: Change to Red/Blue

        int SpeedFactor;
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

        transferServo = hardwareMap.get(CRServo.class, "transferServo");

        SpeedFactor = 1;
        flyWheelAcceleration = 0.06;

        axonPowerLeft.setDirection(Servo.Direction.FORWARD);
        axonPowerRight.setDirection(Servo.Direction.FORWARD);
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

                double leftPos = hood.getLeftServoPosition(distanceToDepot);
                double rightPos = hood.getRightServoPosition(distanceToDepot);

                axonPowerLeft.setPosition(leftPos);
                axonPowerRight.setPosition(rightPos);

                leftBackMotor.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * SpeedFactor);
                leftFrontMotor.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedFactor);
                rightBackMotor.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * SpeedFactor);
                rightFrontMotor.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * SpeedFactor);

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
                    transferServo.setPower(1);
                } else if (gamepad1.circle || gamepad2.a) {
                    transferServo.setPower(-1);
                } else if (gamepad1.triangle) {
                    transferServo.setPower(0.5);
                    intakeMotor.setPower(-1);
                } else {
                    transferServo.setPower(0);
                }

                REALFLYWHEELSPEED = Math.abs(((DcMotorEx) rightHoodMotor).getVelocity());

                telemetry.addData("Flywheel Speed: ", REALFLYWHEELSPEED);
                telemetry.addData("Distance to Depot: ", distanceToDepot);
                telemetry.addData("Left Hood Servo Position: ", leftPos);
                telemetry.addData("Right Hood Servo Position: ", rightPos);
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
}