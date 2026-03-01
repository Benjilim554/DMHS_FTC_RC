package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "NearBlueAuto", group = "Autonomous")
public class NearBlue extends LinearOpMode {

    //TODO: Make sure all hardware configs are correct
    public class intakeIn { // Intake In Class
        private DcMotorEx intakeMotor;

        public intakeIn(HardwareMap hardwareMap) {
            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD); // intakeIn specific

        }
    }
    public class runIntakeIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeIn.setPower(0.5);
            return false;
        }
        public Action runIntakeIn() {
            return new runIntakeIn();
        }

    }


    public class Transfer { // Intake In & Transfer Servo In
        private DcMotorEx intakeMotor;
        private CRServo transferServo;

        public Transfer(HardwareMap hardwareMap) {
            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intakeMotor");
            transferServo = (CRServo) hardwareMap.get(CRServo.class, "transferServo");
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            transferServo.setDirection(CRServo.Direction.FORWARD);

        }
    }

    public class transferWhileShooting { // Intake In, Transfer Servo In, Flywheel In
        private DcMotorEx intakeMotor, rightShooterMotor, leftShooterMotor;
        private CRServo transferServo;

        public transferWhileShooting(HardwareMap hardwareMap) {
            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intakeMotor");
            transferServo = (CRServo) hardwareMap.get(CRServo.class, "transferServo");
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            transferServo.setDirection(CRServo.Direction.FORWARD);
        }

    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        intakeIn intakeMotor = new intakeIn(hardwareMap);
    }
}
