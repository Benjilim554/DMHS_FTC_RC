package org.firstinspires.ftc.teamcode.subsystem.util;

import com.seattlesolvers.solverslib.util.InterpLUT;
import com.qualcomm.robotcore.util.Range;

/**
 * Output: servo position for both hood servos (0.0 to 1.0)
 */
public class HoodLookupTable {

    private final InterpLUT servoLut = new InterpLUT();

    // Safety clamp for servo positions
    public double getServoPosition(double distance) {
        return Range.clip(servoLut.get(distance), 0.0, 1.0);
    }
    public HoodLookupTable() {
        // Distance = inches
        // Format: add(distance, servoPosition)
        servoLut.add(0.00, 1.00);
        servoLut.add(10.00, 1.00);
        servoLut.add(20.00, 0.76);
        servoLut.add(30.00, 0.71);
        servoLut.add(40.00, 0.67); //TODO: Just testing (original 0.78)
        servoLut.add(50.00, 0.63);
        servoLut.add(60.00, 0.58); //TODO: Just testing (original 0.69)
        servoLut.add(70.00, 0.54);
        servoLut.add(80.00, 0.49);
        servoLut.add(90.00, 0.46);
        servoLut.add(100.00, 0.40);
        servoLut.createLUT();
    }
}