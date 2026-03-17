package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystem.util.PIDFController;

@Config
@Autonomous(name = "Trial FarRed", group = "Autonomous")
public class TrialFarRed extends LinearOpMode {

    // Intake / transfer
    private DcMotor intakeMotor;
    private CRServo transferServo;

    // Flywheel motors must be DcMotorEx for getVelocity()
    private DcMotorEx leftHoodMotor;
    private DcMotorEx rightHoodMotor;

    // -------- Dashboard-tunable values --------
    public static double TARGET_VELOCITY = 1500.0;   // ticks/sec, tune this
    public static double VELOCITY_TOLERANCE = 50.0;  // ticks/sec
    public static double SPINUP_TIMEOUT = 2.0;       // seconds

    // PIDF gains (start with your tuned values)
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double kF = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.1338;

    public static double intakeWait = 3.0;

    // -------- Intake actions --------
    public class IntakeOn implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(-0.6);
            transferServo.setPower(1.0);
        }
    }

    public class IntakeOff implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0.0);
            transferServo.setPower(0.0);
        }
    }

    // BALL INTAKE ON & OFF (lower power intakeMotor)
    public class balltakeOn implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(-.7);
            transferServo.setPower(-1);
            leftHoodMotor.setPower(-0.1);
            rightHoodMotor.setPower(-0.1);
        }
    }
    public class balltakeOff implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0);
            transferServo.setPower(0);
            leftHoodMotor.setPower(0);
            rightHoodMotor.setPower(0);
        }
    }

    // -------- Flywheel subsystem --------
    public class Flywheel {
        private final PIDFController controller;

        private double targetVelocity = 0.0;
        private double targetAccel = 0.0;
        private boolean enabled = false;

        public Flywheel() {
            controller = new PIDFController(P, I, D, kF, kV, kA, kS);
        }

        public void setTarget(double velocity, double accel) {
            targetVelocity = velocity;
            targetAccel = accel;
            enabled = true;
        }

        public void stop() {
            enabled = false;
            leftHoodMotor.setPower(0.0);
            rightHoodMotor.setPower(0.0);
        }

        public double getVelocity() {

            return ((leftHoodMotor.getVelocity() + rightHoodMotor.getVelocity()) / 2);
        }

        public boolean atSpeed() {
            return Math.abs(targetVelocity - getVelocity()) <= VELOCITY_TOLERANCE;
        }

        private double computePower() {
            controller.setPIDF(P, I, D, kF);
            controller.setFeedforward(kV, kA, kS);

            double velocity = getVelocity();
            double power = controller.calculate(targetVelocity - velocity, targetVelocity, targetAccel);

            // clip to motor power range
            if (power > 1.0) power = 1.0;
            if (power < -1.0) power = -1.0;
            return power;
        }

        public void update(TelemetryPacket packet) {
            if (!enabled) {
                leftHoodMotor.setPower(0.0);
                rightHoodMotor.setPower(0.0);
                return;
            }

            double velocity = getVelocity();
            double power = computePower();

            leftHoodMotor.setPower(power);
            rightHoodMotor.setPower(power);

            packet.put("flywheelTarget", targetVelocity);
            packet.put("flywheelVelocity", velocity);
            packet.put("flywheelError", targetVelocity - velocity);
            packet.put("flywheelAtSpeed", atSpeed());
            packet.put("flywheelPower", power);
        }

        // Runs forever until the enclosing ParallelAction ends
        public class HoldVelocity implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                update(packet);
                return true;
            }
        }

        // One-shot: enables velocity control
        public class SetTargetAction implements Action {
            private final double vel;
            private final double accel;
            private boolean initialized = false;

            public SetTargetAction(double vel, double accel) {
                this.vel = vel;
                this.accel = accel;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setTarget(vel, accel);
                    initialized = true;
                }
                update(packet);
                return false;
            }
        }

        // Waits until flywheel is ready, but gives up after timeout
        public class WaitUntilReady implements Action {
            private final double timeoutSec;
            private double startTime = -1.0;

            public WaitUntilReady(double timeoutSec) {
                this.timeoutSec = timeoutSec;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (startTime < 0) {
                    startTime = System.nanoTime() / 1e9;
                }

                update(packet);

                boolean timedOut = (System.nanoTime() / 1e9 - startTime) >= timeoutSec;

                return !atSpeed() && !timedOut;
            }
        }

        public class StopAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stop();
                return false;
            }
        }

        public Action setTargetAction(double vel, double accel) {
            return new SetTargetAction(vel, accel);
        }

        public Action holdVelocityAction() {
            return new HoldVelocity();
        }

        public Action waitUntilReadyAction(double timeoutSec) {
            return new WaitUntilReady(timeoutSec);
        }

        public Action stopAction() {
            return new StopAction();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferServo = hardwareMap.get(CRServo.class, "transferServo");

        leftHoodMotor = hardwareMap.get(DcMotorEx.class, "leftHoodMotor");
        rightHoodMotor = hardwareMap.get(DcMotorEx.class, "rightHoodMotor");

        leftHoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d initialPose = new Pose2d(61, 21, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Flywheel flywheel = new Flywheel();

        waitForStart();
        if (isStopRequested()) return;

        Action fullAutoPath = drive.actionBuilder(initialPose)
                // first shot position
                .strafeToLinearHeading(new Vector2d(52, 13.5), Math.toRadians(152))

                // only wait here because we are about to shoot
                .stopAndAdd(flywheel.waitUntilReadyAction(SPINUP_TIMEOUT))
                .stopAndAdd(new SequentialAction(
                        new InstantAction(new IntakeOn()),
                        new SleepAction(intakeWait),
                        new InstantAction(new IntakeOff())
                ))

                // go collect
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(38, 40))
                .afterDisp(0, new balltakeOn())
                .strafeTo(new Vector2d(38, 47))
                .afterDisp(0, new balltakeOff())

                // return to shoot
                .strafeToLinearHeading(new Vector2d(52, 13.5), Math.toRadians(152))
                .stopAndAdd(flywheel.waitUntilReadyAction(0.75)) // usually shorter because it was already spinning
                .stopAndAdd(new SequentialAction(
                        new InstantAction(new IntakeOn()),
                        new SleepAction(intakeWait),
                        new InstantAction(new IntakeOff())
                ))

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        flywheel.holdVelocityAction(),
                        new SequentialAction(
                                flywheel.setTargetAction(TARGET_VELOCITY, 0.0),
                                fullAutoPath,
                                flywheel.stopAction()
                        )
                )
        );
    }
}