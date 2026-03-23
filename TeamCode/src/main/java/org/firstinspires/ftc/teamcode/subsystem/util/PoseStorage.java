package org.firstinspires.ftc.teamcode.subsystem.util;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, 0);

    public static double leftServoStartPosition = 0.0;
    public static double rightServoStartPosition = 0.0;
}