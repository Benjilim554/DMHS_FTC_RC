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
@Autonomous(name = "Full FarRed", group = "Autonomous")
public class FarRed extends LinearOpMode {
    DcMotor intakeMotor, leftHoodMotor, rightHoodMotor;
    CRServo transferServo;

    // INTAKE ON & OFF
    public class intakeOn implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(-.6);
            transferServo.setPower(1);
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
            leftHoodMotor.setPower(.695);
            rightHoodMotor.setPower(.695);
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
        double intakeWait = 3; //TODO: Tune intakeWait
        double balltakeWait = 0;
        double balltakeVelContraint = 65; // 80 = regular velocity

        Pose2d initialPose = new Pose2d(61, 21, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        telemetry.addData("Test","wait");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            telemetry.addData("pass", "wait");
            telemetry.update();
            Action path = drive.actionBuilder(initialPose)
                    .stopAndAdd(new shooterOn()) // change if needed to .afterDisp
                    .strafeToLinearHeading(new Vector2d(52, 13.5), Math.toRadians(152)) // go to shooting zone
                    //  .afterDisp(0, new shooterOn())
                    //TODO: Tune wait for spin-up
                    .waitSeconds(spinWait) // wait for spin-up
                    .stopAndAdd(new SequentialAction(
                            new InstantAction(new intakeOn()),
                            //TODO: Tune intake time
                            new SleepAction(intakeWait), // Feed long enough to shoot
                            // STOP INTAKE & FLYWHEEL
                            new InstantAction(new intakeOff()),
                            new InstantAction(new shooterOff())
                    ))

                    .build();

            Actions.runBlocking(new SequentialAction(path));
        }
    }
}
