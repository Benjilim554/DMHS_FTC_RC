package org.firstinspires.ftc.teamcode.subsystem.util;

import com.seattlesolvers.solverslib.util.InterpLUT;
import com.qualcomm.robotcore.util.Range;

/**
 * Output: servo position for both hood servos (0.0 to 1.0)
 */
public class HoodLookupTable {

    private final InterpLUT servoLut = new InterpLUT();

    // Safety clamp for servo positions
    public double getLeftServoPosition(double distance) {
        return Range.clip(servoLut.get(distance), 0.0, 1.0);
    }

    public double getRightServoPosition(double distance) {
        return Range.clip(servoLut.get(distance), 0.0, 1.0);
    }

    public HoodLookupTable() {
        // Distance = inches
        // Format: add(distance, servoPosition offset)
        servoLut.add(20.0, 0.25);
        servoLut.add(30.0, 0.34);
        servoLut.add(40.0, 0.43);
        servoLut.add(50.0, 0.52);
        servoLut.add(60.0, 0.61);
        servoLut.add(70.0, 0.69);

        servoLut.createLUT();
    }
}