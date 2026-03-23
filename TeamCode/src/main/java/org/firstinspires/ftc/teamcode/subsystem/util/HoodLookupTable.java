package org.firstinspires.ftc.teamcode.subsystem.util;

import com.seattlesolvers.solverslib.util.InterpLUT;
import com.qualcomm.robotcore.util.Range;

/**
 * Distance -> hood servo position lookup using SolversLib InterpLUT.
 *
 * Input: distance to depot/goal (in the same units as your RR field, usually inches)
 * Output: servo position for both hood servos (0.0 to 1.0)
 */
public class HoodLookupTable {

    private final InterpLUT servoLut = new InterpLUT();

    // Safety clamp for servo positions
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    public double getLeftServoPosition(double distance) {
        double offset = servoLut.get(distance);
        double finalPos = PoseStorage.leftServoStartPosition + offset;
        return Range.clip(finalPos, MIN_POS, MAX_POS);
    }

    public double getRightServoPosition(double distance) {
        double offset = servoLut.get(distance);
        double finalPos = PoseStorage.rightServoStartPosition + offset;
        return Range.clip(finalPos, MIN_POS, MAX_POS);
    }

    public HoodLookupTable() {
        // TODO: REPLACE these sample calibration points with your real measured values.
        // Distance = inches
        // Format: add(distance, servoPosition)
        servoLut.add(20.0, 0.25);
        servoLut.add(30.0, 0.34);
        servoLut.add(40.0, 0.43);
        servoLut.add(50.0, 0.52);
        servoLut.add(60.0, 0.61);
        servoLut.add(70.0, 0.69);

        servoLut.createLUT();
    }
}