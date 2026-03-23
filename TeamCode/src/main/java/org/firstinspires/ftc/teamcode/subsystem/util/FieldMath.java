package org.firstinspires.ftc.teamcode.subsystem.util;

import com.acmerobotics.roadrunner.Pose2d;

public class FieldMath {

    public enum Alliance {
        RED,
        BLUE
    }

    private static final double RED_DEPOT_X = 55;
    private static final double RED_DEPOT_Y = 57;

    private static final double BLUE_DEPOT_X = -58;
    private static final double BLUE_DEPOT_Y = -54;

    public static double distanceToDepot(Pose2d pose, double offsetX, Alliance alliance) {
        double targetX;
        double targetY;

        if (alliance == Alliance.RED) {
            targetX = RED_DEPOT_X;
            targetY = RED_DEPOT_Y;
        } else {
            targetX = BLUE_DEPOT_X;
            targetY = BLUE_DEPOT_Y;
        }

        double heading = pose.heading.toDouble();

        double adjustedX = pose.position.x + offsetX;
        double adjustedY = pose.position.y;

        return Math.hypot(adjustedX - targetX, adjustedY - targetY);
    }
}