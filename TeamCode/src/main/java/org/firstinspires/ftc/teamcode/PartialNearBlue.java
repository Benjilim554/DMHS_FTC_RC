package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.InstantFunction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Partial NearBlue", group = "Autonomous")
public class PartialNearBlue extends LinearOpMode {
    DcMotor intakeMotor, leftHoodMotor, rightHoodMotor;
    CRServo transferServo;

    // INTAKE ON & OFF
    public class intakeOn implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(-1);
            transferServo.setPower(.85);
        }
    }
    public class ballIntake implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(-1);
            transferServo.setPower(-1);
        }
    }
    public class intakeOff implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0);
            transferServo.setPower(0);
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
    // SHOOTER ON & OFF
    public class shooterOn implements InstantFunction {
        @Override
        public void run() {
            leftHoodMotor.setPower(.73);
            rightHoodMotor.setPower(.73);
        }
    }
    public class shooterOff implements InstantFunction {
        @Override
        public void run() {
            leftHoodMotor.setPower(-.2);
            rightHoodMotor.setPower(-.2);
        }
    }

    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftHoodMotor = hardwareMap.get(DcMotor.class, "leftHoodMotor");
        rightHoodMotor = hardwareMap.get(DcMotor.class, "rightHoodMotor");
        transferServo = hardwareMap.get(CRServo.class, "transferServo");

        double spinWait = 1.5; //TODO: Tune spinWait, change to 1 if needed
        double intakeWait = 2.4; //TODO: Tune intakeWait
        double balltakeWait = 0;
        double balltakeVelContraint = 90; // 80 = regular velocity

        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(-126));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        Action path = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-16, -13)) // go to shooting zone

                .afterDisp(0, new shooterOn())
                //TODO: Tune wait for spin-up
                .waitSeconds(spinWait) // wait for spin-up
                .stopAndAdd(new SequentialAction (
                        new InstantAction(new intakeOn()),
                        //TODO: Tune intake time
                        new SleepAction(intakeWait), // Feed long enough to shoot
                        // STOP INTAKE & FLYWHEEL
                        new InstantAction(new intakeOff()),
                        new InstantAction(new shooterOff())
                ))

                .strafeToLinearHeading(new Vector2d(-3, -15), Math.toRadians(-88))
                .waitSeconds(balltakeWait)// travel to the front of 1st row of artifacts
                .afterDisp(0, new balltakeOn()) // turn on intake
                .strafeTo(new Vector2d(-3, -54.5), new TranslationalVelConstraint(balltakeVelContraint)) // grab artifacts
                .afterDisp(0, new balltakeOff()) // turn off intake
                .strafeToLinearHeading(new Vector2d(-16, -13), Math.toRadians(-138)) // travel to shooting zone

                .afterDisp(0, new shooterOn())
                //TODO: Tune wait for spin-up
                .waitSeconds(spinWait) // wait for spin-up
                .stopAndAdd(new SequentialAction (
                        new InstantAction(new intakeOn()),
                        //TODO: Tune intake time
                        new SleepAction(intakeWait), // Feed long enough to shoot
                        // STOP INTAKE & FLYWHEEL
                        new InstantAction(new intakeOff()),
                        new InstantAction(new shooterOff())
                ))

                .strafeToLinearHeading(new Vector2d(7, -30), Math.toRadians(-45))

                .build();

        Actions.runBlocking(new SequentialAction(path));
    }
}
